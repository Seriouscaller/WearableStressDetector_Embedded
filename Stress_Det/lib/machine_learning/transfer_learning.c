#include "transfer_learning.h"
#include <stdio.h>

/*
Phase 1:
1. Validate data. Are there too much empty values?
2. Do we have enough different data? Phase variety

Phase 2:
3. Normalize the data using the mix and max of the collected data

Phase 3:
4. Iterative tuning
 - Find BMU of data
 - Nudge the neighborhood
Learning rate starts very tiny, and decays over time

Phase 4:
5. Benchmark the model
 - Calculate Quantization Error, and compare old model with new
 - If error is larger, the model is worse
6. Replace old model
7. Clear the binary data file on the device

Phase 5:
8. Check for custom map in memory. Use new map if found,
   else go back to default.
9. Start inference

*/