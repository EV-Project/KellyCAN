# KellyCAN
Arduino library for monitoring a Kelly Motor Controller

This library includes the stateful messaging system the Kelly controller expects, and unpacks all variables according to the datasheet.

This library relies on a call-back system developed seperately.

The example code cycles through all messages available in the datasheet and populates all local variables with the raw values. (currently broken)
