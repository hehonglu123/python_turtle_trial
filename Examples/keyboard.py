import termios, fcntl, sys, os

#keyboard reading settings
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

print("Running")
print("Press Arrow Key to Control Turtle")
print("Press q to quit")
try:
    while True:
        try:
            #read input and print "command"
            c = sys.stdin.read()
            if "\x1b[A" in c:
                print("drive forward")          ####Drive forward
            if "\x1b[B" in c:
                print("drive backward")         ####Drive backward               
            if "\x1b[C" in c:
                print("drive right")            ####Drive right
            if "\x1b[D" in c:
                print("drive left")             ####Drive left
            if "q" in c:
                break

        except IOError: pass
#finish reading keyboard input
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)