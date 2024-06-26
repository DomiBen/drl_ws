# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from trajectory_planner/GetPoseCmdRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class GetPoseCmdRequest(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "trajectory_planner/GetPoseCmdRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetPoseCmdRequest, self).__init__(*args, **kwds)

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      pass
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from trajectory_planner/GetPoseCmdResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class GetPoseCmdResponse(genpy.Message):
  _md5sum = "5aef3792e74d728815d4dfbf8b1f490f"
  _type = "trajectory_planner/GetPoseCmdResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 result
int32 state
float32 x
float32 y
float32 z
float32 a
float32 b
float32 c
float32 jointAngle_1
float32 jointAngle_2
float32 jointAngle_3
float32 jointAngle_4
float32 jointAngle_5
float32 jointAngle_6
float32 jointAngle_7

"""
  __slots__ = ['result','state','x','y','z','a','b','c','jointAngle_1','jointAngle_2','jointAngle_3','jointAngle_4','jointAngle_5','jointAngle_6','jointAngle_7']
  _slot_types = ['int32','int32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       result,state,x,y,z,a,b,c,jointAngle_1,jointAngle_2,jointAngle_3,jointAngle_4,jointAngle_5,jointAngle_6,jointAngle_7

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetPoseCmdResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = 0
      if self.state is None:
        self.state = 0
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.a is None:
        self.a = 0.
      if self.b is None:
        self.b = 0.
      if self.c is None:
        self.c = 0.
      if self.jointAngle_1 is None:
        self.jointAngle_1 = 0.
      if self.jointAngle_2 is None:
        self.jointAngle_2 = 0.
      if self.jointAngle_3 is None:
        self.jointAngle_3 = 0.
      if self.jointAngle_4 is None:
        self.jointAngle_4 = 0.
      if self.jointAngle_5 is None:
        self.jointAngle_5 = 0.
      if self.jointAngle_6 is None:
        self.jointAngle_6 = 0.
      if self.jointAngle_7 is None:
        self.jointAngle_7 = 0.
    else:
      self.result = 0
      self.state = 0
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.a = 0.
      self.b = 0.
      self.c = 0.
      self.jointAngle_1 = 0.
      self.jointAngle_2 = 0.
      self.jointAngle_3 = 0.
      self.jointAngle_4 = 0.
      self.jointAngle_5 = 0.
      self.jointAngle_6 = 0.
      self.jointAngle_7 = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2i13f().pack(_x.result, _x.state, _x.x, _x.y, _x.z, _x.a, _x.b, _x.c, _x.jointAngle_1, _x.jointAngle_2, _x.jointAngle_3, _x.jointAngle_4, _x.jointAngle_5, _x.jointAngle_6, _x.jointAngle_7))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 60
      (_x.result, _x.state, _x.x, _x.y, _x.z, _x.a, _x.b, _x.c, _x.jointAngle_1, _x.jointAngle_2, _x.jointAngle_3, _x.jointAngle_4, _x.jointAngle_5, _x.jointAngle_6, _x.jointAngle_7,) = _get_struct_2i13f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2i13f().pack(_x.result, _x.state, _x.x, _x.y, _x.z, _x.a, _x.b, _x.c, _x.jointAngle_1, _x.jointAngle_2, _x.jointAngle_3, _x.jointAngle_4, _x.jointAngle_5, _x.jointAngle_6, _x.jointAngle_7))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 60
      (_x.result, _x.state, _x.x, _x.y, _x.z, _x.a, _x.b, _x.c, _x.jointAngle_1, _x.jointAngle_2, _x.jointAngle_3, _x.jointAngle_4, _x.jointAngle_5, _x.jointAngle_6, _x.jointAngle_7,) = _get_struct_2i13f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2i13f = None
def _get_struct_2i13f():
    global _struct_2i13f
    if _struct_2i13f is None:
        _struct_2i13f = struct.Struct("<2i13f")
    return _struct_2i13f
class GetPoseCmd(object):
  _type          = 'trajectory_planner/GetPoseCmd'
  _md5sum = '5aef3792e74d728815d4dfbf8b1f490f'
  _request_class  = GetPoseCmdRequest
  _response_class = GetPoseCmdResponse
