class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

print("Running")
print("Press Arrow Key to Control Turtle")
print("Press q to quit")
getch = _Getch()
while True:
    c=getch()
    if "A" in c:
        print("drive forward")          ####Drive forward
    if "B" in c:
        print("drive backward")         ####Drive backward               
    if "C" in c:
        print("drive right")            ####Drive right
    if "D" in c:
        print("drive left")             ####Drive left
    if 'q' in c:
        break
