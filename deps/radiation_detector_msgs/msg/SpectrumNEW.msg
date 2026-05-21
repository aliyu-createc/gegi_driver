# Defines a Spectrum message produced by the radiation detector

Header header        # timestamp in this header is start of the realTime

time endTime
uint32 realTime_s    # the total duration of the reading in whole seconds (legacy)
uint32 realTime_ms   # the total duration of the reading in milliseconds (use this one)
uint32 deadTime_ms   # how much of the reading time was lost due to being busy... 
uint32 totalCount    # totalnumber of counts observed during reading
uint32[] spectrum    # array of energy bins in which the individual counts were sorted
float64 doserate

