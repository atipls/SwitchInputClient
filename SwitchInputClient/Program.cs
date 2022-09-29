using System.Diagnostics;
using System.IO.Ports;
using SDL2;

namespace SwitchInputClient;

public static class Program {
    const int TICKS_FOR_NEXT_FRAME = (1000 / 60);
    public static unsafe void Main() {
        using var serialPort = new SerialPort("/dev/tty.usbserial-0001", 115200, Parity.None, 8, StopBits.One);
        serialPort.Open();

        var sdlWindow = SDL.SDL_CreateWindow("Switch Input Client", SDL.SDL_WINDOWPOS_CENTERED,
            SDL.SDL_WINDOWPOS_CENTERED, 1280, 720, SDL.SDL_WindowFlags.SDL_WINDOW_SHOWN);

        var lastFrame = DateTime.Now;
        uint lastTick = 0;
        int lastHash = 0;
        while (true) {
            SDL.SDL_PumpEvents();
            while (lastTick - SDL.SDL_GetTicks() < TICKS_FOR_NEXT_FRAME) {
                SDL.SDL_Delay(1);
            }
            
            var packet = new SwitchPacket();
            SDL.SDL_CaptureMouse(SDL.SDL_bool.SDL_TRUE);
            SDL.SDL_SetRelativeMouseMode(SDL.SDL_bool.SDL_TRUE);

            var keyboardState = (byte*) SDL.SDL_GetKeyboardState(out var numKeys);

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_ESCAPE] != 0) {
                break;
            }
            
            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_W] != 0) {
                packet.LStickY = 0;
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_S] != 0) {
                packet.LStickY = 255;
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_A] != 0) {
                packet.LStickX = 0;
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_D] != 0) {
                packet.LStickX = 255;
            }
            
            var mouseState = SDL.SDL_GetRelativeMouseState(out var mouseX, out var mouseY);
            packet.RStickX = (byte) (Math.Clamp(mouseX * 3, -128, 127) + 127);
            packet.RStickY = (byte) (Math.Clamp(mouseY * 3, -128, 127) + 127);
            
            if (packet.RStickX != 127 || packet.RStickY != 127) {
                Console.WriteLine($"X: {packet.RStickX-127}, Y: {packet.RStickY-127}");
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_UP] != 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_LEFT] == 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_RIGHT] == 0) {
                packet.DPad = DPad.Up;
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_UP] != 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_LEFT] != 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_RIGHT] == 0) {
                packet.DPad = DPad.UpLeft;
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_UP] != 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_LEFT] == 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_RIGHT] != 0) {
                packet.DPad = DPad.UpRight;
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_DOWN] != 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_LEFT] == 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_RIGHT] == 0) {
                packet.DPad = DPad.Down;
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_DOWN] != 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_LEFT] != 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_RIGHT] == 0) {
                packet.DPad = DPad.DownLeft;
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_DOWN] != 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_LEFT] == 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_RIGHT] != 0) {
                packet.DPad = DPad.DownRight;
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_LEFT] != 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_UP] == 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_DOWN] == 0) {
                packet.DPad = DPad.Left;
            }

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_RIGHT] != 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_UP] == 0 &&
                keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_DOWN] == 0) {
                packet.DPad = DPad.Right;
            }
            
            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_RETURN] != 0)
                packet.MSB |= MSBButtons.A;

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_SPACE] != 0)
                packet.MSB |= MSBButtons.B;

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_R] != 0)
                packet.MSB |= MSBButtons.X;

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_F] != 0)
                packet.MSB |= MSBButtons.Y;
            
            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_Q] != 0)
                packet.MSB |= MSBButtons.L;
            
            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_E] != 0)
                packet.MSB |= MSBButtons.R;
            
            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_LSHIFT] != 0)
                packet.LSB |= LSBButtons.LStick;

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_LCTRL] != 0)
                packet.LSB |= LSBButtons.RStick;
            
            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_TAB] != 0)
                packet.LSB |= LSBButtons.Plus;
            
            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_BACKSPACE] != 0)
                packet.LSB |= LSBButtons.Minus;
            
            
            // Mouse1 - ZR
            // Mouse2 - ZL
            
            if ((mouseState & SDL.SDL_BUTTON_LMASK) != 0)
                packet.MSB |= MSBButtons.ZR;
            
            if ((mouseState & SDL.SDL_BUTTON_RMASK) != 0)
                packet.MSB |= MSBButtons.ZL;

            if (serialPort.BytesToRead > 0)
                //serialPort.ReadExisting();
                Console.Write(serialPort.ReadExisting());

            if (lastHash == packet.GetHashCode()) {
                lastTick = SDL.SDL_GetTicks();
                continue;
            }
            
            //Console.WriteLine("Time since last frame: " + (DateTime.Now - lastFrame).TotalMilliseconds);

            var serialized = packet.Serialize();
            serialPort.Write(serialized, 0, serialized.Length);
            lastFrame = DateTime.Now;
            lastHash = packet.GetHashCode();
            lastTick = SDL.SDL_GetTicks();

            // Swap buffers
            //SDL.SDL_Delay(15);
        }
    }
}