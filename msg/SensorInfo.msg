uint32 id               # base pair position
string label            # custom user set label
uint32 type             # sensor type (hardware-specific, cf. Trigno SDK Manual)
uint32 mode             # configured operating mode (type-specific, cf. Trigno SDK Manual)
string serial
string firmware
uint32 n_emg_channels
uint32 n_aux_channels
float32 sample_rate
string units
float32 gain
bool range              # low vs high range
bool bandwidth          # narrow vs wide bandwidth
