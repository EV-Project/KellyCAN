# KellyCAN
Adruino library for monitoring a Kelly Motor Controller

This library includes the stateful messaging system the Kelly controller expects, and unpacks all variables according to the datasheet.

This library also offers a basic wrapper around the mcp2515 library for a Sparkfun CAN bus shield.

The example code cycles through all messages available in the datasheet and populates all local variables with the raw values.
