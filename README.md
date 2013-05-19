## How to compile

1. Get ns-allinone-2.31.tar.gz and extract it
2. Install git, e.g. in Debian: ```sudo apt-get install git```
3. Clone this repository: ```git clone git://github.com/abdulla-alali/TFRC-CR.git -b TFRCCR```
4. You'll end up with a ns-2.31 clone that has the CRAHN and TFRCCR modules integrated. Replace the ns-2.31 subfolder 
(from the ns-allinone file that you already extracted) with the one you cloned from here.
5. ```./configure``` and then ```make```

in TCL
----
    TFRC_CR set-repository $repository

where repository is 

    set repository [new CrossLayerRepository]

    TFRC_CR here is: [new Agent/TFRC_CR]

    TFRC_CR set-pu-model $pumap
    
where pumap is 

    set pumap [new PUMap]

    TFRC_CR_Sink set binMultiplier <multiplier M number>
    TFRC_CR_Sink here is [new Agent/TFRC_CR_Sink]
