# PortAudio configurations are used for this message.
# Refer to https://www.portaudio.com/docs/v19-doxydocs/portaudio_8h.html

uint32 PA_INT16 = 8

std_msgs/Header header
uint32 sample_format    # The Unreal Engine outputs data in int16 format so this will always be paInt16
uint32 num_channels     # Number of audio channels (1 for mono or 2 for stereo)
uint32 sample_rate      # Number of samples per second (ex. 1600)
uint32 chunk_size       # Size of each audio frame (ex. 4096)
int16[] data            # Audio data