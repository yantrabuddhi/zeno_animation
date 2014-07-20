import bpy
import os
import sys

dirname = os.path.dirname(bpy.data.filepath)
filename = os.path.join(dirname, "ZenoAnimation.py")

sys.path.insert(0, dirname)
exec(compile(open(filename).read(), filename, 'exec'))
