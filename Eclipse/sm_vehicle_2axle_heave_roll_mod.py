# This file was automatically generated by SWIG (https://www.swig.org).
# Version 4.2.0
#
# Do not make changes to this file unless you know what you are doing - modify
# the SWIG interface file instead.

from sys import version_info as _swig_python_version_info
# Import the low-level C/C++ module
if __package__ or "." in __name__:
    from . import _sm_vehicle_2axle_heave_roll_mod
else:
    import _sm_vehicle_2axle_heave_roll_mod

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)


def _swig_setattr_nondynamic_instance_variable(set):
    def set_instance_attr(self, name, value):
        if name == "this":
            set(self, name, value)
        elif name == "thisown":
            self.this.own(value)
        elif hasattr(self, name) and isinstance(getattr(type(self), name), property):
            set(self, name, value)
        else:
            raise AttributeError("You cannot add instance attributes to %s" % self)
    return set_instance_attr


def _swig_setattr_nondynamic_class_variable(set):
    def set_class_attr(cls, name, value):
        if hasattr(cls, name) and not isinstance(getattr(cls, name), property):
            set(cls, name, value)
        else:
            raise AttributeError("You cannot add class attributes to %s" % cls)
    return set_class_attr


def _swig_add_metaclass(metaclass):
    """Class decorator for adding a metaclass to a SWIG wrapped class - a slimmed down version of six.add_metaclass"""
    def wrapper(cls):
        return metaclass(cls.__name__, cls.__bases__, cls.__dict__.copy())
    return wrapper


class _SwigNonDynamicMeta(type):
    """Meta class to enforce nondynamic attributes (no new attributes) for a class"""
    __setattr__ = _swig_setattr_nondynamic_class_variable(type.__setattr__)


class B_sm_vehicle_2axle_heave_roll_T(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    Ackermanleft = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_Ackermanleft_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_Ackermanleft_set)
    INPUT_6_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_6_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_6_1_1_set)
    Ackermanright = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_Ackermanright_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_Ackermanright_set)
    INPUT_5_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_5_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_5_1_1_set)
    INPUT_7_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_7_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_7_1_1_set)
    INPUT_8_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_8_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_8_1_1_set)
    INPUT_9_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_9_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_9_1_1_set)
    INPUT_12_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_12_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_12_1_1_set)
    INPUT_10_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_10_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_10_1_1_set)
    INPUT_11_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_11_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_11_1_1_set)
    INPUT_13_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_13_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_13_1_1_set)
    INPUT_14_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_14_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_14_1_1_set)
    INPUT_15_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_15_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_15_1_1_set)
    INPUT_18_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_18_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_18_1_1_set)
    INPUT_16_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_16_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_16_1_1_set)
    INPUT_17_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_17_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_17_1_1_set)
    INPUT_19_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_19_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_19_1_1_set)
    INPUT_20_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_20_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_20_1_1_set)
    INPUT_21_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_21_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_21_1_1_set)
    INPUT_24_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_24_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_24_1_1_set)
    INPUT_22_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_22_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_22_1_1_set)
    INPUT_23_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_23_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_23_1_1_set)
    INPUT_25_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_25_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_25_1_1_set)
    INPUT_26_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_26_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_26_1_1_set)
    INPUT_27_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_27_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_27_1_1_set)
    INPUT_30_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_30_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_30_1_1_set)
    INPUT_28_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_28_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_28_1_1_set)
    INPUT_29_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_29_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_29_1_1_set)
    STATE_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_STATE_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_STATE_1_set)
    INPUT_4_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_4_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_4_1_1_set)
    INPUT_1_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_1_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_1_1_1_set)
    INPUT_3_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_3_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_3_1_1_set)
    INPUT_2_1_1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_2_1_1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_INPUT_2_1_1_set)
    TrigonometricFunction1 = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_TrigonometricFunction1_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_TrigonometricFunction1_set)
    Flipsignforxaxis = property(_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_Flipsignforxaxis_get, _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_Flipsignforxaxis_set)

    def __init__(self):
        _sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_swiginit(self, _sm_vehicle_2axle_heave_roll_mod.new_B_sm_vehicle_2axle_heave_roll_T())
    __swig_destroy__ = _sm_vehicle_2axle_heave_roll_mod.delete_B_sm_vehicle_2axle_heave_roll_T

# Register B_sm_vehicle_2axle_heave_roll_T in _sm_vehicle_2axle_heave_roll_mod:
_sm_vehicle_2axle_heave_roll_mod.B_sm_vehicle_2axle_heave_roll_T_swigregister(B_sm_vehicle_2axle_heave_roll_T)
class DW_sm_vehicle_2axle_heave_rol_T(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    INPUT_6_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_6_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_6_1_1_Discrete_set)
    INPUT_6_1_1_FirstOutput = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_6_1_1_FirstOutput_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_6_1_1_FirstOutput_set)
    INPUT_5_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_5_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_5_1_1_Discrete_set)
    INPUT_5_1_1_FirstOutput = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_5_1_1_FirstOutput_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_5_1_1_FirstOutput_set)
    INPUT_7_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_7_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_7_1_1_Discrete_set)
    INPUT_8_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_8_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_8_1_1_Discrete_set)
    INPUT_9_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_9_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_9_1_1_Discrete_set)
    INPUT_12_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_12_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_12_1_1_Discrete_set)
    INPUT_10_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_10_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_10_1_1_Discrete_set)
    INPUT_11_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_11_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_11_1_1_Discrete_set)
    INPUT_13_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_13_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_13_1_1_Discrete_set)
    INPUT_14_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_14_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_14_1_1_Discrete_set)
    INPUT_15_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_15_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_15_1_1_Discrete_set)
    INPUT_18_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_18_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_18_1_1_Discrete_set)
    INPUT_16_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_16_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_16_1_1_Discrete_set)
    INPUT_17_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_17_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_17_1_1_Discrete_set)
    INPUT_19_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_19_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_19_1_1_Discrete_set)
    INPUT_20_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_20_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_20_1_1_Discrete_set)
    INPUT_21_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_21_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_21_1_1_Discrete_set)
    INPUT_24_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_24_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_24_1_1_Discrete_set)
    INPUT_22_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_22_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_22_1_1_Discrete_set)
    INPUT_23_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_23_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_23_1_1_Discrete_set)
    INPUT_25_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_25_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_25_1_1_Discrete_set)
    INPUT_26_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_26_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_26_1_1_Discrete_set)
    INPUT_27_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_27_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_27_1_1_Discrete_set)
    INPUT_30_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_30_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_30_1_1_Discrete_set)
    INPUT_28_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_28_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_28_1_1_Discrete_set)
    INPUT_29_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_29_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_29_1_1_Discrete_set)
    INPUT_4_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_4_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_4_1_1_Discrete_set)
    INPUT_1_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_1_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_1_1_1_Discrete_set)
    INPUT_3_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_3_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_3_1_1_Discrete_set)
    INPUT_2_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_2_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_INPUT_2_1_1_Discrete_set)
    STATE_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_Discrete_set)
    OUTPUT_1_0_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_Discrete_set)
    OUTPUT_1_1_Discrete = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_Discrete_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_Discrete_set)
    STATE_1_Simulator = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_Simulator_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_Simulator_set)
    STATE_1_SimData = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_SimData_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_SimData_set)
    STATE_1_DiagMgr = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_DiagMgr_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_DiagMgr_set)
    STATE_1_ZcLogger = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_ZcLogger_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_ZcLogger_set)
    STATE_1_TsInfo = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_TsInfo_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_TsInfo_set)
    OUTPUT_1_0_Simulator = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_Simulator_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_Simulator_set)
    OUTPUT_1_0_SimData = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_SimData_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_SimData_set)
    OUTPUT_1_0_DiagMgr = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_DiagMgr_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_DiagMgr_set)
    OUTPUT_1_0_ZcLogger = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_ZcLogger_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_ZcLogger_set)
    OUTPUT_1_0_TsInfo = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_TsInfo_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_TsInfo_set)
    SINK_1_RtwLogger = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_SINK_1_RtwLogger_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_SINK_1_RtwLogger_set)
    SINK_1_RtwLogBuffer = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_SINK_1_RtwLogBuffer_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_SINK_1_RtwLogBuffer_set)
    SINK_1_RtwLogFcnManager = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_SINK_1_RtwLogFcnManager_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_SINK_1_RtwLogFcnManager_set)
    OUTPUT_1_1_Simulator = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_Simulator_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_Simulator_set)
    OUTPUT_1_1_SimData = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_SimData_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_SimData_set)
    OUTPUT_1_1_DiagMgr = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_DiagMgr_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_DiagMgr_set)
    OUTPUT_1_1_ZcLogger = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_ZcLogger_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_ZcLogger_set)
    OUTPUT_1_1_TsInfo = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_TsInfo_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_TsInfo_set)
    STATE_1_Modes = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_Modes_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_Modes_set)
    OUTPUT_1_0_Modes = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_Modes_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_Modes_set)
    OUTPUT_1_1_Modes = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_Modes_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_Modes_set)
    STATE_1_FirstOutput = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_FirstOutput_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_STATE_1_FirstOutput_set)
    OUTPUT_1_0_FirstOutput = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_FirstOutput_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_0_FirstOutput_set)
    OUTPUT_1_1_FirstOutput = property(_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_FirstOutput_get, _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_OUTPUT_1_1_FirstOutput_set)

    def __init__(self):
        _sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_swiginit(self, _sm_vehicle_2axle_heave_roll_mod.new_DW_sm_vehicle_2axle_heave_rol_T())
    __swig_destroy__ = _sm_vehicle_2axle_heave_roll_mod.delete_DW_sm_vehicle_2axle_heave_rol_T

# Register DW_sm_vehicle_2axle_heave_rol_T in _sm_vehicle_2axle_heave_roll_mod:
_sm_vehicle_2axle_heave_roll_mod.DW_sm_vehicle_2axle_heave_rol_T_swigregister(DW_sm_vehicle_2axle_heave_rol_T)
class X_sm_vehicle_2axle_heave_roll_T(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    sm_vehicle_2axle_heave_rollVehi = property(_sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_sm_vehicle_2axle_heave_rollVehi_get, _sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_sm_vehicle_2axle_heave_rollVehi_set)
    sm_vehicle_2axle_heave_rollVe_d = property(_sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_sm_vehicle_2axle_heave_rollVe_d_get, _sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_sm_vehicle_2axle_heave_rollVe_d_set)
    sm_vehicle_2axle_heave_rollVe_o = property(_sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_sm_vehicle_2axle_heave_rollVe_o_get, _sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_sm_vehicle_2axle_heave_rollVe_o_set)
    TransferFcn_CSTATE = property(_sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_TransferFcn_CSTATE_get, _sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_TransferFcn_CSTATE_set)
    TransferFcn1_CSTATE = property(_sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_TransferFcn1_CSTATE_get, _sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_TransferFcn1_CSTATE_set)

    def __init__(self):
        _sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_swiginit(self, _sm_vehicle_2axle_heave_roll_mod.new_X_sm_vehicle_2axle_heave_roll_T())
    __swig_destroy__ = _sm_vehicle_2axle_heave_roll_mod.delete_X_sm_vehicle_2axle_heave_roll_T

# Register X_sm_vehicle_2axle_heave_roll_T in _sm_vehicle_2axle_heave_roll_mod:
_sm_vehicle_2axle_heave_roll_mod.X_sm_vehicle_2axle_heave_roll_T_swigregister(X_sm_vehicle_2axle_heave_roll_T)
class XDot_sm_vehicle_2axle_heave_r_T(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    sm_vehicle_2axle_heave_rollVehi = property(_sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVehi_get, _sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVehi_set)
    sm_vehicle_2axle_heave_rollVe_d = property(_sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVe_d_get, _sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVe_d_set)
    sm_vehicle_2axle_heave_rollVe_o = property(_sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVe_o_get, _sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVe_o_set)
    TransferFcn_CSTATE = property(_sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_TransferFcn_CSTATE_get, _sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_TransferFcn_CSTATE_set)
    TransferFcn1_CSTATE = property(_sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_TransferFcn1_CSTATE_get, _sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_TransferFcn1_CSTATE_set)

    def __init__(self):
        _sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_swiginit(self, _sm_vehicle_2axle_heave_roll_mod.new_XDot_sm_vehicle_2axle_heave_r_T())
    __swig_destroy__ = _sm_vehicle_2axle_heave_roll_mod.delete_XDot_sm_vehicle_2axle_heave_r_T

# Register XDot_sm_vehicle_2axle_heave_r_T in _sm_vehicle_2axle_heave_roll_mod:
_sm_vehicle_2axle_heave_roll_mod.XDot_sm_vehicle_2axle_heave_r_T_swigregister(XDot_sm_vehicle_2axle_heave_r_T)
class XDis_sm_vehicle_2axle_heave_r_T(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    sm_vehicle_2axle_heave_rollVehi = property(_sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVehi_get, _sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVehi_set)
    sm_vehicle_2axle_heave_rollVe_d = property(_sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVe_d_get, _sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVe_d_set)
    sm_vehicle_2axle_heave_rollVe_o = property(_sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVe_o_get, _sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_sm_vehicle_2axle_heave_rollVe_o_set)
    TransferFcn_CSTATE = property(_sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_TransferFcn_CSTATE_get, _sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_TransferFcn_CSTATE_set)
    TransferFcn1_CSTATE = property(_sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_TransferFcn1_CSTATE_get, _sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_TransferFcn1_CSTATE_set)

    def __init__(self):
        _sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_swiginit(self, _sm_vehicle_2axle_heave_roll_mod.new_XDis_sm_vehicle_2axle_heave_r_T())
    __swig_destroy__ = _sm_vehicle_2axle_heave_roll_mod.delete_XDis_sm_vehicle_2axle_heave_r_T

# Register XDis_sm_vehicle_2axle_heave_r_T in _sm_vehicle_2axle_heave_roll_mod:
_sm_vehicle_2axle_heave_roll_mod.XDis_sm_vehicle_2axle_heave_r_T_swigregister(XDis_sm_vehicle_2axle_heave_r_T)
class ODE1_IntgData(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    f = property(_sm_vehicle_2axle_heave_roll_mod.ODE1_IntgData_f_get, _sm_vehicle_2axle_heave_roll_mod.ODE1_IntgData_f_set)

    def __init__(self):
        _sm_vehicle_2axle_heave_roll_mod.ODE1_IntgData_swiginit(self, _sm_vehicle_2axle_heave_roll_mod.new_ODE1_IntgData())
    __swig_destroy__ = _sm_vehicle_2axle_heave_roll_mod.delete_ODE1_IntgData

# Register ODE1_IntgData in _sm_vehicle_2axle_heave_roll_mod:
_sm_vehicle_2axle_heave_roll_mod.ODE1_IntgData_swigregister(ODE1_IntgData)
class ExtU_sm_vehicle_2axle_heave_r_T(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    u = property(_sm_vehicle_2axle_heave_roll_mod.ExtU_sm_vehicle_2axle_heave_r_T_u_get, _sm_vehicle_2axle_heave_roll_mod.ExtU_sm_vehicle_2axle_heave_r_T_u_set)

    def __init__(self):
        _sm_vehicle_2axle_heave_roll_mod.ExtU_sm_vehicle_2axle_heave_r_T_swiginit(self, _sm_vehicle_2axle_heave_roll_mod.new_ExtU_sm_vehicle_2axle_heave_r_T())
    __swig_destroy__ = _sm_vehicle_2axle_heave_roll_mod.delete_ExtU_sm_vehicle_2axle_heave_r_T

# Register ExtU_sm_vehicle_2axle_heave_r_T in _sm_vehicle_2axle_heave_roll_mod:
_sm_vehicle_2axle_heave_roll_mod.ExtU_sm_vehicle_2axle_heave_r_T_swigregister(ExtU_sm_vehicle_2axle_heave_r_T)
class ExtY_sm_vehicle_2axle_heave_r_T(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    y = property(_sm_vehicle_2axle_heave_roll_mod.ExtY_sm_vehicle_2axle_heave_r_T_y_get, _sm_vehicle_2axle_heave_roll_mod.ExtY_sm_vehicle_2axle_heave_r_T_y_set)

    def __init__(self):
        _sm_vehicle_2axle_heave_roll_mod.ExtY_sm_vehicle_2axle_heave_r_T_swiginit(self, _sm_vehicle_2axle_heave_roll_mod.new_ExtY_sm_vehicle_2axle_heave_r_T())
    __swig_destroy__ = _sm_vehicle_2axle_heave_roll_mod.delete_ExtY_sm_vehicle_2axle_heave_r_T

# Register ExtY_sm_vehicle_2axle_heave_r_T in _sm_vehicle_2axle_heave_roll_mod:
_sm_vehicle_2axle_heave_roll_mod.ExtY_sm_vehicle_2axle_heave_r_T_swigregister(ExtY_sm_vehicle_2axle_heave_r_T)
class tag_RTM_sm_vehicle_2axle_heav_T(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr
    errorStatus = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_errorStatus_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_errorStatus_set)
    solverInfo = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_solverInfo_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_solverInfo_set)
    blockIO = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_blockIO_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_blockIO_set)
    contStates = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_contStates_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_contStates_set)
    periodicContStateIndices = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_periodicContStateIndices_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_periodicContStateIndices_set)
    periodicContStateRanges = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_periodicContStateRanges_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_periodicContStateRanges_set)
    derivs = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_derivs_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_derivs_set)
    contStateDisabled = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_contStateDisabled_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_contStateDisabled_set)
    zCCacheNeedsReset = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_zCCacheNeedsReset_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_zCCacheNeedsReset_set)
    derivCacheNeedsReset = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_derivCacheNeedsReset_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_derivCacheNeedsReset_set)
    CTOutputIncnstWithState = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_CTOutputIncnstWithState_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_CTOutputIncnstWithState_set)
    odeF = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_odeF_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_odeF_set)
    intgData = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_intgData_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_intgData_set)
    dwork = property(_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_dwork_get, _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_dwork_set)

    def __init__(self):
        _sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_swiginit(self, _sm_vehicle_2axle_heave_roll_mod.new_tag_RTM_sm_vehicle_2axle_heav_T())
    __swig_destroy__ = _sm_vehicle_2axle_heave_roll_mod.delete_tag_RTM_sm_vehicle_2axle_heav_T

# Register tag_RTM_sm_vehicle_2axle_heav_T in _sm_vehicle_2axle_heave_roll_mod:
_sm_vehicle_2axle_heave_roll_mod.tag_RTM_sm_vehicle_2axle_heav_T_swigregister(tag_RTM_sm_vehicle_2axle_heav_T)

def sm_vehicle_2axle_heave_roll_initialize(sm_vehicle_2axle_heave_roll_M, sm_vehicle_2axle_heave_roll_U, sm_vehicle_2axle_heave_roll_Y):
    return _sm_vehicle_2axle_heave_roll_mod.sm_vehicle_2axle_heave_roll_initialize(sm_vehicle_2axle_heave_roll_M, sm_vehicle_2axle_heave_roll_U, sm_vehicle_2axle_heave_roll_Y)

def sm_vehicle_2axle_heave_roll_step(sm_vehicle_2axle_heave_roll_M, sm_vehicle_2axle_heave_roll_Y):
    return _sm_vehicle_2axle_heave_roll_mod.sm_vehicle_2axle_heave_roll_step(sm_vehicle_2axle_heave_roll_M, sm_vehicle_2axle_heave_roll_Y)

def sm_vehicle_2axle_heave_roll_terminate(sm_vehicle_2axle_heave_roll_M):
    return _sm_vehicle_2axle_heave_roll_mod.sm_vehicle_2axle_heave_roll_terminate(sm_vehicle_2axle_heave_roll_M)

cvar = _sm_vehicle_2axle_heave_roll_mod.cvar
sm_vehicle_2axle_heave_roll_rtZ = cvar.sm_vehicle_2axle_heave_roll_rtZ
sm_vehicle_2axle_heave_roll_r_0 = cvar.sm_vehicle_2axle_heave_roll_r_0

