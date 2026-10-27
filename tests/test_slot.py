"""Validation tests for the stator-slot thermal model (thermal/slot_model.py)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from thermal.slot_model import MATRIX, Geometry, annulus_check, bundle_k, keff_fd, r_wh  # noqa: E402


def test_fd_solver_matches_heated_annulus():
    fd, exact = annulus_check(h=0.1e-3)
    assert abs(fd / exact - 1) < 0.01


def test_bundle_formula_matches_wire_array_fd():
    for km in MATRIX.values():
        for phi in (0.3, 0.45):
            f = bundle_k(phi, km, enamel_ratio=0.05)
            s = keff_fd(phi, km, enamel_ratio=0.05, n=120)
            assert abs(f / s - 1) < 0.05


def test_potting_lowers_and_bare_raises_r_wh():
    g = Geometry()
    air, var, epo = (r_wh(g, m, h=0.1e-3) for m in ("air", "varnish", "epoxy"))
    assert air > var > epo
    assert air < 3.55 * 1.2          # bare winding still below ~the datasheet total
