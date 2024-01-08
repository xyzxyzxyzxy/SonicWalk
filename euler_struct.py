from ctypes import Structure, c_double
class Euler(Structure):
    
    def __eq__(self, other):
        for fld in self._fields_:
            if getattr(self, fld[0]) != getattr(other, fld[0]):
                return False
        return True

    def __ne__(self, other):
        for fld in self._fields_:
            if getattr(self, fld[0]) != getattr(other, fld[0]):
                return True
        return False

    _fields_ = [("roll", c_double), ("pitch", c_double), ("yaw", c_double)]