# -----------------------------------------------------------------------------
# This file was autogenerated by symforce from template:
#     function/FUNCTION.py.jinja
# Do NOT modify by hand.
# -----------------------------------------------------------------------------

# pylint: disable=too-many-locals,too-many-lines,too-many-statements,unused-argument,unused-import

import math
import typing as T

import numpy

import sym


def fuse_airspeed(v_local, state, P, airspeed, R, epsilon):
    # type: (numpy.ndarray, numpy.ndarray, numpy.ndarray, float, float, float) -> T.Tuple[numpy.ndarray, numpy.ndarray, float, float]
    """
    This function was autogenerated from a symbolic function. Do not modify by hand.

    Symbolic function: fuse_airspeed

    Args:
        v_local: Matrix31
        state: Matrix31
        P: Matrix33
        airspeed: Scalar
        R: Scalar
        epsilon: Scalar

    Outputs:
        H: Matrix13
        K: Matrix31
        innov_var: Scalar
        innov: Scalar
    """

    # Total ops: 56

    # Input arrays
    if v_local.shape == (3,):
        v_local = v_local.reshape((3, 1))
    elif v_local.shape != (3, 1):
        raise IndexError(
            "v_local is expected to have shape (3, 1) or (3,); instead had shape {}".format(
                v_local.shape
            )
        )

    if state.shape == (3,):
        state = state.reshape((3, 1))
    elif state.shape != (3, 1):
        raise IndexError(
            "state is expected to have shape (3, 1) or (3,); instead had shape {}".format(
                state.shape
            )
        )

    # Intermediate terms (11)
    _tmp0 = -state[0, 0] + v_local[0, 0]
    _tmp1 = -state[1, 0] + v_local[1, 0]
    _tmp2 = math.sqrt(_tmp0**2 + _tmp1**2 + epsilon + v_local[2, 0] ** 2)
    _tmp3 = state[2, 0] / _tmp2
    _tmp4 = _tmp0 * _tmp3
    _tmp5 = _tmp1 * _tmp3
    _tmp6 = -P[0, 0] * _tmp4
    _tmp7 = -P[1, 1] * _tmp5
    _tmp8 = P[2, 2] * _tmp2
    _tmp9 = (
        R
        + _tmp2 * (-P[0, 2] * _tmp4 - P[1, 2] * _tmp5 + _tmp8)
        - _tmp4 * (-P[1, 0] * _tmp5 + P[2, 0] * _tmp2 + _tmp6)
        - _tmp5 * (-P[0, 1] * _tmp4 + P[2, 1] * _tmp2 + _tmp7)
    )
    _tmp10 = 1 / max(_tmp9, epsilon)

    # Output terms
    _H = numpy.zeros(3)
    _H[0] = -_tmp4
    _H[1] = -_tmp5
    _H[2] = _tmp2
    _K = numpy.zeros(3)
    _K[0] = _tmp10 * (-P[0, 1] * _tmp5 + P[0, 2] * _tmp2 + _tmp6)
    _K[1] = _tmp10 * (-P[1, 0] * _tmp4 + P[1, 2] * _tmp2 + _tmp7)
    _K[2] = _tmp10 * (-P[2, 0] * _tmp4 - P[2, 1] * _tmp5 + _tmp8)
    _innov_var = _tmp9
    _innov = -_tmp2 * state[2, 0] + airspeed
    return _H, _K, _innov_var, _innov
