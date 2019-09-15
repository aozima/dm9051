from building import *

cwd  = GetCurrentDir()
src  = Glob('src/*.c')
CPPPATH = [cwd]
objs = []

group = DefineGroup('DM9051', src, depend = [''], CPPPATH = CPPPATH)

Return('group')
