import hexapod_class

Briggs = hexapod_class.hexapod()

Briggs.stand()
Briggs.gait(60, "ripple", 400) # Ripple gait 400mm in a direction 60 degrees from X+
Briggs.sit()
Briggs.stopPWM()




    
