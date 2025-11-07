echo "Finding RFT device"

# Start candump in background
candump can0 > can_output.log &
CANDUMP_PID=$!

# Give candump time to start
sleep 2

# Increase TX queue length
sudo ifconfig can0 txqueuelen 10000
cansend can0 0f8#0100000000000000

a=0
while [ $a -lt 2047 ]; do
    hex_id=$(printf "%03X" $a)
    echo "Sending to ID: $hex_id"
    
    cansend can0 ${hex_id}#0100000000000000
    
    a=$((a + 1))
    # sleep 0.1  # Reduce sleep time
done

# Stop candump
kill $CANDUMP_PID 2>/dev/null


