# 4x230V-Triac-Dimmer
based on Waveshare Triac Hat
can be found here: https://www.waveshare.com/2-ch-triac-hat.htm

But with some modifications:
1. Remove snupper: R2/R6 and C1/C2 to avoid climming LED lamps in off mode
2. Remove Raspi hat (40 pins) use I2C connector only
3. If you want to control more than one hat at the same time, make a current save mod:
   Replace R3 and R7 with 681 Ohm and Replace optocoppler Moc3021 with Moc3023
   
Dual triac mode is possible on I2C adress 0x47 and 0x46 (or 0x43)
