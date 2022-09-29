namespace SwitchInputClient;

[Flags]
public enum MSBButtons : byte {
    Y = 1 << 0,
    B = 1 << 1,
    A = 1 << 2,
    X = 1 << 3,
    L = 1 << 4,
    R = 1 << 5,
    ZL = 1 << 6,
    ZR = 1 << 7,
}

[Flags]
public enum LSBButtons : byte {
    Minus = 1 << 0,
    Plus = 1 << 1,
    LStick = 1 << 2,
    RStick = 1 << 3,
    Home = 1 << 4,
    Capture = 1 << 5,
    SL = 1 << 6,
    SR = 1 << 7,
}

public enum DPad : byte {
    Up = 0,
    UpRight = 1,
    Right = 2,
    DownRight = 3,
    Down = 4,
    DownLeft = 5,
    Left = 6,
    UpLeft = 7,
    Neutral = 8,
}

public struct SwitchPacket {
    public LSBButtons LSB;
    public MSBButtons MSB;
    public DPad DPad;
    public byte LStickX = 128;
    public byte LStickY = 128;
    public byte RStickX = 128;
    public byte RStickY = 128;
    public byte Unused;

    public SwitchPacket() {
        MSB = 0;
        LSB = 0;
        DPad = DPad.Neutral;
        Unused = 0;
    }

    public byte UpdateCRC8(byte oldCrc, byte newData) {
        byte data = (byte) (oldCrc ^ newData);
        for (int i = 0; i < 8; i++) {
            if ((data & 0x80) != 0) {
                data = (byte) (data << 1);
                data = (byte) (data ^ 0x07);
            }
            else {
                data <<= 1;
            }

            data &= 0xFF;
        }

        return data;
    }

    public byte CalculateCRC8() {
        byte crc = 0;
        crc = UpdateCRC8(crc, (byte) LSB);
        crc = UpdateCRC8(crc, (byte) MSB);
        crc = UpdateCRC8(crc, (byte) DPad);
        crc = UpdateCRC8(crc, LStickX);
        crc = UpdateCRC8(crc, LStickY);
        crc = UpdateCRC8(crc, RStickX);
        crc = UpdateCRC8(crc, RStickY);
        crc = UpdateCRC8(crc, Unused);
        return crc;
    }

    public byte[] Serialize() {
        return new[] {
            (byte) LSB,
            (byte) MSB,
            (byte) DPad,
            LStickX,
            LStickY,
            RStickX,
            RStickY,
            Unused,
            CalculateCRC8(),
        };
    }

    public override int GetHashCode() {
        int hash1 = 0, hash2 = 0;
        hash1 |= (int) LSB;
        hash1 |= (int) MSB << 8;
        hash1 |= (int) DPad << 16;
        hash1 |= (int) LStickX << 24;
        hash2 |= (int) LStickY;
        hash2 |= (int) RStickX << 8;
        hash2 |= (int) RStickY << 16;
        hash2 |= (int) Unused << 24;
        return hash1 ^ hash2;
    }
}