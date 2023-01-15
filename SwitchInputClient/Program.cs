using System.Diagnostics;
using System.IO.Ports;
using SDL2;

namespace SwitchInputClient;

public static class Program {
    private const int TICKS_FOR_NEXT_FRAME = 1000 / 60;

    private static unsafe void HandleKeyboard(ref SwitchPacket packet) {
         var keyboardState = (byte*) SDL.SDL_GetKeyboardState(out var numKeys);

            if (keyboardState[(int) SDL.SDL_Scancode.SDL_SCANCODE_ESCAPE] != 0) {
                //break;
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
            
            mouseX = Math.Clamp(mouseX, -127, 127);
            mouseY = Math.Clamp(mouseY, -127, 127);
            
            const int mouseScale = 1;
            var mouseXScaled = (sbyte) (mouseX / mouseScale);
            var mouseYScaled = (sbyte) (mouseY / mouseScale);
            
            packet.RStickX = (byte) (mouseXScaled + 127);
            packet.RStickY = (byte) (mouseYScaled + 127);
            
            if (packet.RStickX != 127 || packet.RStickY != 127) {
                Console.WriteLine($"X: {packet.RStickX}, Y: {packet.RStickY}");
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
            
            if ((mouseState & SDL.SDL_BUTTON_LMASK) != 0)
                packet.MSB |= MSBButtons.ZR;
            
            if ((mouseState & SDL.SDL_BUTTON_RMASK) != 0)
                packet.MSB |= MSBButtons.ZL;
    }

    private static void HandleController(ref SwitchPacket packet, IntPtr controller) {
        // Hardcoded for the Xbox Series X controller
        
        var leftX = SDL.SDL_GameControllerGetAxis(controller, SDL.SDL_GameControllerAxis.SDL_CONTROLLER_AXIS_LEFTX);
        var leftY = SDL.SDL_GameControllerGetAxis(controller, SDL.SDL_GameControllerAxis.SDL_CONTROLLER_AXIS_LEFTY);
        var rightX = SDL.SDL_GameControllerGetAxis(controller, SDL.SDL_GameControllerAxis.SDL_CONTROLLER_AXIS_RIGHTX);
        var rightY = SDL.SDL_GameControllerGetAxis(controller, SDL.SDL_GameControllerAxis.SDL_CONTROLLER_AXIS_RIGHTY);
        
        var leftTrigger = SDL.SDL_GameControllerGetAxis(controller, SDL.SDL_GameControllerAxis.SDL_CONTROLLER_AXIS_TRIGGERLEFT) > 0;
        var rightTrigger = SDL.SDL_GameControllerGetAxis(controller, SDL.SDL_GameControllerAxis.SDL_CONTROLLER_AXIS_TRIGGERRIGHT) > 0;
        
        // Remap from -32768 to 32767 to 0 to 255, 127 is the center
        
        packet.LStickX = (byte) ((leftX + 32768) / 256);
        packet.LStickY = (byte) ((leftY + 32768) / 256);
        packet.RStickX = (byte) ((rightX + 32768) / 256);
        packet.RStickY = (byte) ((rightY + 32768) / 256);

        packet.MSB |= leftTrigger ? MSBButtons.ZL : 0;
        packet.MSB |= rightTrigger ? MSBButtons.ZR : 0;
        
        var dpadUp = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_DPAD_UP) != 0;
        var dpadDown = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_DPAD_DOWN) != 0;
        var dpadLeft = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_DPAD_LEFT) != 0;
        var dpadRight = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_DPAD_RIGHT) != 0;
        
        if (dpadUp && dpadLeft) {
            packet.DPad = DPad.UpLeft;
        } else if (dpadUp && dpadRight) {
            packet.DPad = DPad.UpRight;
        } else if (dpadDown && dpadLeft) {
            packet.DPad = DPad.DownLeft;
        } else if (dpadDown && dpadRight) {
            packet.DPad = DPad.DownRight;
        } else if (dpadUp) {
            packet.DPad = DPad.Up;
        } else if (dpadDown) {
            packet.DPad = DPad.Down;
        } else if (dpadLeft) {
            packet.DPad = DPad.Left;
        } else if (dpadRight) {
            packet.DPad = DPad.Right;
        } else {
            packet.DPad = DPad.Neutral;
        }
        
        var a = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_A) != 0;
        var b = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_B) != 0;
        var x = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_X) != 0;
        var y = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_Y) != 0;
        
        // A -> B
        // B -> A
        // X -> Y
        // Y -> X
        packet.MSB |= a ? MSBButtons.B : 0;
        packet.MSB |= b ? MSBButtons.A : 0;
        
        packet.MSB |= x ? MSBButtons.Y : 0;
        packet.MSB |= y ? MSBButtons.X : 0;
        
        var leftShoulder = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_LEFTSHOULDER) != 0;
        var rightShoulder = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER) != 0;
        
        packet.MSB |= leftShoulder ? MSBButtons.L : 0;
        packet.MSB |= rightShoulder ? MSBButtons.R : 0;
        
        var leftStick = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_LEFTSTICK) != 0;
        var rightStick = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_RIGHTSTICK) != 0;
        
        packet.LSB |= leftStick ? LSBButtons.LStick : 0;
        packet.LSB |= rightStick ? LSBButtons.RStick : 0;


        var minus = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_BACK) != 0;
        var plus = SDL.SDL_GameControllerGetButton(controller, SDL.SDL_GameControllerButton.SDL_CONTROLLER_BUTTON_START) != 0;
        
        packet.LSB |= minus ? LSBButtons.Minus : 0;
        packet.LSB |= plus ? LSBButtons.Plus : 0;
    }
    
    public static void Main() {
        SDL.SDL_Init(SDL.SDL_INIT_VIDEO | SDL.SDL_INIT_JOYSTICK | SDL.SDL_INIT_GAMECONTROLLER);
        
        using var serialPort = new SerialPort("/dev/tty.usbserial-0001", 115200, Parity.None, 8, StopBits.One);
        serialPort.Open();

        var sdlWindow = SDL.SDL_CreateWindow("Switch Input Client", SDL.SDL_WINDOWPOS_CENTERED,
            SDL.SDL_WINDOWPOS_CENTERED, 1280, 720, SDL.SDL_WindowFlags.SDL_WINDOW_SHOWN);

        var controller = SDL.SDL_GameControllerOpen(0);
        
        Console.WriteLine($"Using controller: {SDL.SDL_GameControllerName(controller)}");
        
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

           HandleKeyboard(ref packet);
           HandleController(ref packet, controller);

            if (serialPort.BytesToRead > 0)
                Console.Write(serialPort.ReadExisting());

            
            
            if (lastHash == packet.GetHashCode()) {
                lastTick = SDL.SDL_GetTicks();
                continue;
            }
            
            //Console.WriteLine("Time since last frame: " + (DateTime.Now - lastFrame).TotalMilliseconds);

            var serialized = packet.Serialize();
            serialPort.Write(serialized, 0, serialized.Length);

            lastHash = packet.GetHashCode();
            lastTick = SDL.SDL_GetTicks();

            // Swap buffers
            //SDL.SDL_Delay(15);
        }
    }
}