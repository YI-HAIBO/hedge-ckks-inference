#!/usr/bin/env python3
# round65v33 Vivado-parse preflight gate (xvlog + xelab), ILA-aware.
#
# What this is
#   The pre-synthesis gate for HEDGE. It runs Vivado's own front-end over the
#   exact RTL SRC_LIST used by hedge_implement.tcl, so a green result means
#   synth_design will not die on parse/elaboration.
#
# Why r33 exists (delta vs rtl_preflight_r22.py)
#   Round65v14 made `ila_bist u_ila_bist` unconditional in hedges_top.v
#   (Vivado 2026.1 removed every Tcl path for `+define+`). Inside the Vivado
#   project the ILA core comes from create_ip, so the module resolves. OUTSIDE
#   the project -- which is exactly what this gate is -- nothing declares
#   module ila_bist, so xelab died with
#       [VRFC 10-2063] Module <ila_bist> not found ... hedges_top.v:1151
#   r22 therefore always reported rc=1 for a reason that says nothing about our
#   RTL. r33 resolves it the same way the iverilog gate does: it pulls in
#   hedge-tb/ila_bist_sim_stub.v and adds a Stage 0 that checks the stub's
#   probe widths against the generated IP netlist, so a future probe change
#   cannot silently break this gate.
#
# HARD RULE - read before touching SRC_LIST
#   ila_bist_sim_stub.v is a SIMULATION-ONLY shim. It must NEVER be added to
#   the SRC_LIST in hedge_implement.tcl: in the real project the ILA IP and
#   the stub would both declare `module ila_bist`, producing a duplicate
#   definition. This script is the ONLY consumer of the stub.
#
# Runtime: ~40 s total. Run BEFORE any Vivado project run -- a synth_design
# elaboration failure costs a full project-open cycle (60 s+), this pinpoints
# the exact file and line in under a minute.
import json, re, subprocess, time
from pathlib import Path

ROOT = Path(r"C:\Users\Yi\WorkBuddy\2026-10-03-09-14-19")
VIV_BIN = Path(r"D:\FPGA\2026.1\Vivado\bin")
# Must be the .bat wrappers: bare `xvlog` is a shell script and raises
# OSError [WinError 193] under subprocess on Windows.
XVLOG = VIV_BIN / "xvlog.bat"
XELAB = VIV_BIN / "xelab.bat"
RTL_DIR = ROOT / "hedge-rtl"
TB_DIR = ROOT / "hedge-tb"
OUT_DIR = ROOT / "hedge-synthes" / "preflight_r33"
OUT_DIR.mkdir(parents=True, exist_ok=True)

TOP_MODULE = "hedges_top"

# Must mirror hedge_implement.tcl SRC_LIST exactly.
SRC_REL = [
    "hedges_top.v",
    "sub_a_plus_c/ac_modules_real.sv",
    "sub_a_plus_c/ac_adapter.v",
    "sub_a_plus_c/nptt_stream_pack.v",
    "sub_a_plus_c/nptt_stream_unpack.v",
    "sub_a_plus_c/keyswitch_pipe.v",
    "sub_a_plus_c/rescale_mod.v",
    "sub_a_plus_c/data_feed.v",
    "sub_a_plus_c/axi_data_load.v",
    "sub_a_plus_c/nptt_wrapper.v",
    "sub_a_plus_c/nptt_top.v",
    "sub_a_plus_c/nptt_cross.v",
    "sub_a_plus_c/ntt32_core_s.v",
    "sub_a_plus_c/nptt_engine_core.v",
    "sub_a_plus_c/nptt_twist.v",
    "sub_a_plus_c/nptt_scatter.v",
    "sub_a_plus_c/nptt_unscatter.v",
    "sub_a_plus_c/nptt_intt_top.v",
    "sub_a_plus_c/nptt_intt_wrapper.v",
    "sub_a_plus_c/ntt32_core.v",
    "sub_a_plus_c/ntt64_core.v",
    "sub_a_plus_c/ntt64_core_s.v",
    "sub_io_bridge/io_bridge.v",
    "sub_io_bridge/cmd_decoder.v",
    "sub_io_bridge/axi_hp_to_stream.v",
    "sub_io_bridge/axi4_rd_master.v",
    "sub_io_bridge/ver_check.v",
    "sub_layer_scheduler/layer_fsm.v",
    "sub_layer_scheduler/layer_scheduler.v",
    "sub_nonlinear_approximator/nonlinear_approximator.v",
    "sub_nonlinear_approximator/relu_coeffs_rom.v",
    "sub_nonlinear_approximator/relu_poly_eval.v",
    "sub_plaintext_ciphertext_bridge/plaintext_ciphertext_bridge.v",
    "sub_plaintext_ciphertext_bridge/softmax_engine.v",
    "sub_plaintext_ciphertext_bridge/argmax_network.v",
    "bist_engine.v",
]
# Sim-only ILA shim -- OUT OF SCOPE for the real project (see HARD RULE).
ILA_STUB = TB_DIR / "ila_bist_sim_stub.v"
ILA_NETLIST = ROOT / "hedge-synthes" / "ila_ip" / "ila_bist" / "synth" / "ila_bist.v"

abs_files = [str(RTL_DIR / f) for f in SRC_REL]
if ILA_STUB not in [Path(p) for p in abs_files]:
    abs_files.append(str(ILA_STUB))
missing = [f for f in abs_files if not Path(f).exists()]
if missing:
    print("MISSING FILES:")
    for m in missing:
        print("  ", m)
    raise SystemExit(2)

BUILD = OUT_DIR / "build"
BUILD.mkdir(parents=True, exist_ok=True)
WORKLIB = "work"


def run(cmd, cwd, log_path, timeout=420):
    t0 = time.time()
    try:
        p = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True,
                           encoding="utf-8", errors="replace", timeout=timeout)
        rc, text = p.returncode, (p.stdout or "") + (("\n--- STDERR ---\n" + p.stderr) if p.stderr else "")
    except subprocess.TimeoutExpired as e:
        rc, text = -1, f"TIMEOUT after {timeout}s\n{e.output or ''}"
    elapsed = time.time() - t0
    log_path.write_text(text, encoding="utf-8", errors="replace")
    return rc, elapsed, text


def classify(text):
    errors, warns = [], []
    for i, ln in enumerate(text.splitlines(), 1):
        s = ln.strip()
        if "ERROR" in s:
            errors.append((i, s))
        elif "CRITICAL WARNING" in s or "WARNING" in s:
            warns.append((i, s))
    return errors, warns


def probe_widths(text):
    """Return {probeN: (hi, lo)} from an ila_bist module declaration.

    Handles both declaration styles found in the repo:
      IP netlist  : `input [5 : 0] probe0;`
      sim stub    : `input  wire [5:0]  probe0,`
    so the type name (wire/reg/logic) between `input` and the range is optional.
    """
    out = {}
    for ln in text.splitlines():
        s = ln.strip()
        if "input" not in s:
            continue
        m = re.search(r"\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*probe\s*(\d+)\s*[,;]?\s*$", s)
        if m:
            out[f"probe{m.group(3)}"] = (int(m.group(1)), int(m.group(2)))
    return out


print("=" * 70)
print("HEDGE round65v33 preflight (xvlog + xelab, ILA-aware)")
print(f"  files  : {len(abs_files)} RTL sources + 1 sim-only ILA stub")
print(f"  top    : {TOP_MODULE}")
print("=" * 70)

clean = True

# ---- Stage 0: ILA stub width check (no Vivado involved) ----
print("\n" + "-" * 70)
print("[Stage 0] checking ila_bist_sim_stub.v against the generated IP netlist ...")
stub_w = probe_widths(ILA_STUB.read_text(encoding="utf-8", errors="replace"))
stub_ok = True
if not ILA_NETLIST.exists():
    stub_ok = False
    clean = False
    print("  ERROR: MISSING IP netlist:", ILA_NETLIST)
else:
    ip_w = probe_widths(ILA_NETLIST.read_text(encoding="utf-8", errors="replace"))
    if not stub_w or not ip_w:
        stub_ok = False
        clean = False
        print("  ERROR: could not parse probe widths (stub=%s, ip=%s)" % (stub_w, ip_w))
    else:
        mismatch = {p: (stub_w.get(p), ip_w.get(p))
                    for p in set(stub_w) | set(ip_w)
                    if stub_w.get(p) != ip_w.get(p)}
        if mismatch:
            stub_ok = False
            clean = False
            print("  ERROR: probe width mismatch between stub and IP:")
            for p, (a, b) in sorted(mismatch.items()):
                print(f"    {p}: stub={a}  ip={b}")
        else:
            print(f"  OK: {len(stub_w)} probes match the IP "
                  f"({', '.join(f'{p}=[{hi}:{lo}]' for p, (hi, lo) in sorted(stub_w.items()))})")

# ---- Stage 1: xvlog ----
print("\n" + "-" * 70)
print("[Stage 1] xvlog -- per-file parse (Verilog 2001 for .v, -sv for .sv) ...")
sv_files = [f for f in abs_files if f.lower().endswith(".sv")]
v_files = [f for f in abs_files if not f.lower().endswith(".sv")]
e1, w1 = [], []
rc1, el1 = 0, 0.0
txt1 = ""
for mode_flag, group, label in [(None, v_files, "verilog"), ("-sv", sv_files, "systemverilog")]:
    if not group:
        continue
    cmd = [str(XVLOG), "--work", WORKLIB]
    if mode_flag:
        cmd.append(mode_flag)
    cmd += group
    rc, el, txt = run(cmd, cwd=BUILD, log_path=OUT_DIR / f"xvlog_r33_{label}.log")
    rc1 = rc1 or rc
    el1 += el
    txt1 += txt
    e, w = classify(txt)
    e1 += e
    w1 += w
print(f"[Stage 1 xvlog] rc={rc1}  {el1:.1f}s  errors={len(e1)}  warns={len(w1)}  "
      f"(verilog={len(v_files)}, systemverilog={len(sv_files)})")
for i, s in e1[:30]:
    print(f"  L{i}: {s}")
for i, s in w1[:10]:
    print(f"  WARN L{i}: {s}")
if rc1 != 0 or e1:
    clean = False
    print("!! Stage 1 failed -- fix parse errors before xelab is meaningful.")

# ---- Stage 2: xelab ----
rc2, el2, e2, w2 = None, 0.0, [], []
txt2 = ""
if clean:
    print("\n" + "-" * 70)
    print("[Stage 2] xelab -- link + elaborate hedges_top ...")
    inc = RTL_DIR / "sub_a_plus_c"  # include dir for ntt_twiddles.vh
    cmd2 = [str(XELAB), f"{WORKLIB}.{TOP_MODULE}",
            "-s", "r33_top", "--debug", "typical", "-i", str(inc)]
    rc2, el2, txt2 = run(cmd2, cwd=BUILD, log_path=OUT_DIR / "xelab_r33.log")
    e2, w2 = classify(txt2)
    print(f"[Stage 2 xelab] rc={rc2}  {el2:.1f}s  errors={len(e2)}  warns={len(w2)}")
    for i, s in e2[:30]:
        print(f"  L{i}: {s}")
    if rc2 != 0 or e2:
        clean = False

summary = {
    "top": TOP_MODULE,
    "round": "round65v33",
    "note": "ILA resolved via sim-only stub; NOT to be added to hedge_implement.tcl",
    "stage0_ila_stub_width_check": {
        "stub_probes": stub_w,
        "ip_probes": probe_widths(ILA_NETLIST.read_text(encoding="utf-8", errors="replace")) if ILA_NETLIST.exists() else None,
    },
    "stage1_xvlog": {"rc": rc1, "n_errors": len(e1), "n_warns": len(w1),
                     "errors": [{"line": i, "msg": s} for i, s in e1]},
    "stage2_xelab": ({"rc": rc2, "n_errors": len(e2), "n_warns": len(w2),
                      "errors": [{"line": i, "msg": s} for i, s in e2]} if rc2 is not None
                     else {"skipped": True}),
    "files": abs_files,
    "clean": clean,
}
(OUT_DIR / "preflight_r33_summary.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

print("\n" + "=" * 70)
if clean:
    print("RESULT: CLEAN - stub widths match the IP; xvlog + xelab passed;")
    print("          synth_design elaboration should succeed.")
else:
    why = []
    if not stub_ok:
        why.append("stage0 stub-width check failed")
    if rc1 != 0 or e1:
        why.append(f"{len(e1)} xvlog error(s)")
    if rc2 is not None and (rc2 != 0 or e2):
        why.append(f"{len(e2)} xelab error(s)")
    if rc2 is None and clean is False and "stage0" not in why[0]:
        why.append("xelab skipped (upstream stage failed)")
    print("RESULT: FAIL -- " + "; ".join(why))
print(f"summary  : {OUT_DIR / 'preflight_r33_summary.json'}")
raise SystemExit(0 if clean else 1)
