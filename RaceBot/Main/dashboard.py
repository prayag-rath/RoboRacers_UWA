"""dashboard.py - the window on the touch display (own process). Started by main.py.

main.py draws the picture and decides what a tap does. This only shows the pictures it gets
on stdin and prints its size and every tap on stdout: "width height", then "x y" per tap.
"""
import os
import sys

DISPLAY = ":0"    # the touch display


def main():
    os.environ["DISPLAY"] = DISPLAY
    os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"    # stdout is for main.py
    import pygame

    pygame.display.init()
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN | pygame.NOFRAME)    # the size of the display
    pygame.mouse.set_visible(False)
    size = screen.get_size()
    print(*size, flush=True)

    n = size[0] * size[1] * 3
    while True:
        frame = sys.stdin.buffer.read(n)
        if len(frame) < n:    # main.py is gone
            break
        screen.blit(pygame.image.frombuffer(frame, size, "RGB"), (0, 0))
        pygame.display.flip()
        for e in pygame.event.get():
            if e.type == pygame.MOUSEBUTTONDOWN:
                print(*e.pos, flush=True)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
