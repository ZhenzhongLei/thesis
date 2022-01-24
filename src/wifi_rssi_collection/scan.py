from wifi_rssi_collection.utils import *

def getAPinfo(output):
    """
    Extract AP info from iwlist output

    Args:
        output, string 

    Returns:
        A list of dictionary with 3 fields: "ssid", "quality" and "signal"
    """
    cells = output.split('Cell') # Divide raw string into raw cells.
    cells.pop(0) # Remove unneccesary "Scan Completed" message.
    if(len(cells) > 0): # Continue execution, if atleast one network is detected.
        def parseCell(cell):
            ssid = getSSID(cell)
            quality = getQuality(cell)
            signal = getSignalLevel(cell)

            if ssid == '':
                AP = None
            else:
                AP = {'ssid': ssid, 'quality': quality, 'signal': signal}
            return AP
        
        def getSSID(cell):
            ssid = cell.split('ESSID:"')[1]
            ssid = ssid.split('"')[0]
            return ssid
        
        def getQuality(cell):
            quality = cell.split('Quality=')[1]
            quality = quality.split(' ')[0]
            return quality           

        def getSignalLevel(cell):
            signal = cell.split('Signal level=')[1]
            signal = signal.split(' ')[0]
            return signal

        APinfo = []
        for cell in cells:
            AP = parseCell(cell)
            if AP is None:
                pass
            else:
                APinfo.append(AP)
        return APinfo
    else:
        print("Networks not detected.")
        return None


def getRawNetworkScan(interface, password = 'luxc1', sudo = False):
    """
    Get raw network data by using iwlist command

    Args:
        interface: string, the network interface which is obtained by using command "iwconfig" from command line
        sudo: if enabled, the scanning will cover wireless networks among the area

    Returns:
        A dictionary with keys "output" and "error", which stores scanning result and error information respectively.
    """
    # Scan command 'iwlist interface scan' needs to be fed as an array.
    if sudo:
        command = ['sudo', '-S', 'iwlist', interface,'scan'] # -S will make Python read password from standard input
    else:
        command = ['iwlist', interface,'scan']

    process, output, error = runSudoCommand(command, password)
    # Block all execution, until the scanning completes.
    process.wait()
    # Returns all output in a dictionary for easy retrieval.
    return {'output':process.stdout.read(),'error':process.stderr.read()}


def runSudoCommand(command, password, input=PIPE, output=PIPE, err=PIPE, verbose=True):
    """
    Run sudo command by using Popen

    Args:
        command: list of strings
        password: string, password
        input: IO object, the input stream to subprocess
        output: IO object, the output stream from subprocess
        err: IO object, the error stream from subprocess
    Returns:
        process object
    """
    if verbose:
        print(reduce(lambda x, y: x + ' ' + y, command))
    # Open a subprocess running the scan command.
    process = Popen(command, stdin=input, stdout=output, stderr=err)
    # Returns the 'success' and 'error' output.
    process.communicate(input=str.encode(password + '\n')) # input has to be byte type
    return process


def getAPdata(interface, password, channels, interval = 0.1):
    """
    Get AP and RSSI data by having sniffer sniffing packets from given interface

    Args:
        interface: string, operable interface, which could be checked by using "iwlist" command
        password: string, password for sudo
        channels: list of integers, representing different frequencies which could be checked by "iwlist interface channel"
        interval: float, the amount of time to operate on different frequencies
    Returns:
        process object
    """
    def configureMonitorMode(interface):
        print("The mode swithcing process runs at " + str(os.getpid()))
        # Turn network manager off
        runSudoCommand(['sudo', '-S', 'service', 'network-manager', 'stop'], password).wait()
        # Turn interface off
        runSudoCommand(['sudo', '-S', 'ifconfig', interface, 'down'], password).wait()
        # Change to monitor mode
        runSudoCommand(['sudo','-S','iwconfig', interface,'mode', 'monitor'], password).wait()
        # Turn interface on
        runSudoCommand(['sudo', '-S', 'ifconfig', interface, 'up'], password).wait()

    def hooping(interface, channels, interval = 0.1):
        global state
        print("The hopping process runs at " + str(os.getpid()))
        while True: 
            for channel in channels:
                runSudoCommand(['sudo', '-S', 'iwconfig', interface, 'channel', str(channel)], password).wait()
                time.sleep(interval) 

    def filterOutput(input):
        print("The filtering process runs at " + str(os.getpid()))
        while True:
            line = input.readline()
            signal_pos  = line.find('signal')
            bssid_pos   = line.find('BSSID')
            channel_pos = line.find('MHz')
            
            if (signal_pos != -1) and (bssid_pos != -1) and (channel_pos != -1):            
                dB = line[:signal_pos].split()[-1][:-2]  # db is block before 'signal' [:-2] so dB is not printed
                channel = line[:channel_pos].split()[-1] #channel is the block before 'MHz'
                BSSID = line[bssid_pos:].split()[0][6:] # BSSID: begins with 'BSSID' [6:] so BSSID is not printed
                #print(str(dB)+' '+str(BSSID)+' '+str(channel), file=sys.stdout)
                #sys.stdout.flush()

    # Configure interface to monitor mode
    configureMonitorMode(interface)

    # Have network 
    thread = threading.Thread(target = hooping, args=[interface, channels, interval])
    thread.daemon = True
    thread.start()
    
    # Receive packets 
    time.sleep(0.1)
    
    command = 'sudo -S tcpdump -npeqi {} -f type mgt subtype beacon'.format(interface)
    sniffing_process = runSudoCommand(command.split(), password)
    sniffing_output = sniffing_process.stdout
    
    # Extract RSSI and APs 
    thread = threading.Thread(target = filterOutput, args=[sniffing_output])
    thread.daemon = True
    thread.start()
