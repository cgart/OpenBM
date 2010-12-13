#!/bin/bash

FILE="Customers.ini"
OUTPUT="customer.h"

# Function: get_config_list config_file
get_config_list()
{
   typeset config_file=$1
   awk -F '[][]' 'NF==3 && $0 ~ /^\[.*\]/ { print $2 }' ${config_file}
}

# Prase config file
cfg.parser ()
{
      IFS=$'\n' && ini=( $(<$1) ) # convert to line-array
      ini=( ${ini[*]//;*/} )      # remove comments �;�
      ini=( ${ini[*]//\#*/} )     # remove comments �#�
      ini=( ${ini[*]/\        =/=} )  # remove tabs before =
      ini=( ${ini[*]/=\       /=} )   # remove tabs be =
      ini=( ${ini[*]/\ =\ /=} )   # remove anything with a space around � = �
      ini=( ${ini[*]/#[/\}$'\n'cfg.section.} ) # set section prefix
      ini=( ${ini[*]/%]/ \(} )    # convert text2function (1)
      ini=( ${ini[*]/=/=\( } )    # convert item to array
      ini=( ${ini[*]/%/ \)} )     # close array parenthesis
      ini=( ${ini[*]/%\\ \)/ \\} ) # the multiline trick
      ini=( ${ini[*]/%\( \)/\(\) \{} ) # convert text2function (2)
      ini=( ${ini[*]/%\} \)/\}} ) # remove extra parenthesis
      ini[0]="" # remove first element
      ini[${#ini[*]} + 1]='}'    # add the last brace
      eval "$(echo "${ini[*]}")" # eval the result
}

# Prase customers config file
cfg.parser ${FILE}

cfg=$1

# for each section generate a makefile
#for cfg in $(get_config_list ${FILE})
#do
   echo "--- Configuration [${cfg}] ---"
   eval "cfg.section."${cfg}

   # print some information about customer's device
   echo "Customer: " $surname, $name
   echo "Device ID: " $DEVID_1 $DEVID_2 $DEVID_3 $DEVID_4
   echo "Voltage Settings: " $DEVID_5 $DEVID_6 $DEVID_7 $DEVID_8 $DEVID_9 $DEVID_10
   echo "Device Codings: " $DEVID_11 $DEVID_12

   # generate config file, which customer specific
   rm -f $OUTPUT
   touch $OUTPUT
   echo "//-------------------------------------------------------------------------" >> $OUTPUT
   echo "//    Automatically generated customer header file, do not change          " >> $OUTPUT
   echo "//-------------------------------------------------------------------------" >> $OUTPUT
   echo " " >> $OUTPUT
   echo "#define CUSTOMER \""${cfg}"\"" >> $OUTPUT
   echo " " >> $OUTPUT
   echo "// splitted device id" >> $OUTPUT 
   echo "#define DEVID_1 0x"$DEVID_1 >> $OUTPUT
   echo "#define DEVID_2 0x"$DEVID_2 >> $OUTPUT
   echo "#define DEVID_3 0x"$DEVID_3 >> $OUTPUT
   echo "#define DEVID_4 0x"$DEVID_4 >> $OUTPUT
   echo "#define DEVID_5 0x"$DEVID_5 >> $OUTPUT
   echo "#define DEVID_6 0x"$DEVID_6 >> $OUTPUT
   echo "#define DEVID_7 0x"$DEVID_7 >> $OUTPUT
   echo "#define DEVID_8 0x"$DEVID_8 >> $OUTPUT
   echo "#define DEVID_9 0x"$DEVID_9 >> $OUTPUT
   echo "#define DEVID_10 0x"$DEVID_10 >> $OUTPUT
   echo "#define DEVID_11 0x"$DEVID_11 >> $OUTPUT
   echo "#define DEVID_12 0x"$DEVID_12 >> $OUTPUT
   echo " " >> $OUTPUT
   echo "// full device id" >> $OUTPUT 
   echo "#define DEVID \""$DEVID_1$DEVID_2$DEVID_3$DEVID_4$DEVID_5$DEVID_6$DEVID_7$DEVID_8$DEVID_9$DEVID_10$DEVID_11$DEVID_12"\"" >> $OUTPUT
   
#done



