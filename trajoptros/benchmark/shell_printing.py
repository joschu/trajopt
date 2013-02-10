import subprocess
def call_and_print(cmd,color='green'):
    print colorize(cmd, color, bold=True)
    subprocess.check_call(cmd, shell=True)
def Popen_and_print(cmd,color='green'):
    print colorize(cmd, color, bold=True)
    return subprocess.Popen(cmd, shell=True)
color2num = dict(
    gray=30,
    red=31,
    green=32,
    yellow=33,
    blue=34,
    magenta=35,
    cyan=36,
    white=37,
    crimson=38
)        
def colorize(string, color, bold=False, highlight = False):
    attr = []
    num = color2num[color]
    if highlight: num += 10
    attr.append(str(num))
    if bold: attr.append('1')
    return '\x1b[%sm%s\x1b[0m' % (';'.join(attr), string)
