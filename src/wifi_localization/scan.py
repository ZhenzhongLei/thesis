from wifi_localization.utils import *

def getScanIwlist(interface, password = 'luxc1'):
    """
    Get raw network data by using iwlist

    Args:
        interface: string, the network interface 
        password: the pass code used to enable sudo

    Returns:
        A list of dictionary with 3 fields: "ssid", "quality" and "signal"
    """
    
    command = "sudo -S iwlist {} scan".format(interface)
    process = runSudoCommand(command.split(), password)
    return {'output':process.stdout.read(),'error':process.stderr.read()}

def processIwlistReturn(output):
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
    
def getScanWpa(interface, password = 'luxc1'):
    """
    Get raw network data by using wpa_supplicant service

    Args:
        interface: string, the network interface 
        password: the pass code used to enable sudo

    Returns:
        A dictionary with keys "output" and "error", which stores scanning result and error information respectively.
    """
    command = "sudo -S wpa_cli -i {} scan".format(interface)
    runSudoCommand(command.split(), password)
    command = "sudo -S wpa_cli -i {} scan_results".format(interface)
    process = runSudoCommand(command.split(), password)
    return {'output':process.stdout.read(),'error':process.stderr.read()}
    
def processWpaReturn(output):
    """
    Extract AP info from wpa_supplicant output

    Args:
        output, string by bytes 

    Returns:
        A list of dictionary with 3 fields: "ssid", "quality" and "signal"
    """
    lines = output.split('\n')
    length = len(lines)
    AP_info = []
    for i in range(1, length):
        fields = lines[i].split('\t')
        if len(fields) < 5:
            continue
        bssid = fields[0]
        signal = fields[2]
        ssid = fields[-1]
        AP = {'bssid': bssid, 'signal': signal, 'ssid': ssid}
        AP_info.append(AP)
    return AP_info

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
    # Input password
    process._stdin_write(input=str.encode(password + '\n')) # Method communicate will block the execution.
    return process


def sniffAPdata(interface, password, channels, interval = 0.1):
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
        """
        Set the specified wifi interface to monitor mode
        """
        runSudoCommand(['sudo', '-S', 'airmon-ng', 'check', 'kill'], password).wait()
        runSudoCommand(['sudo', '-S', 'airmon-ng', 'start', interface], password).wait()
        # runSudoCommand(['sudo', '-S', 'service', 'network-manager', 'stop'], password).wait()
        # runSudoCommand(['sudo', '-S', 'ifconfig', interface, 'down'], password).wait()
        # runSudoCommand(['sudo','-S','iwconfig', interface,'mode', 'monitor'], password).wait()
        # runSudoCommand(['sudo', '-S', 'ifconfig', interface, 'up'], password).wait()

    def hooping(interface, channels, interval = 0.1):
        """
        Change the wifi channel (to receive packets from different frequencies)
        """
        while True: 
            for channel in channels:
                runSudoCommand(['sudo', '-S', 'iwconfig', interface, 'channel', str(channel)], password, verbose=False).wait()
                time.sleep(interval) 

    def filterOutput(input):
        print("The filtering process runs at " + str(os.getpid()))
        encoding = 'utf-8'
        while True:
            line = input.readline()
            signal_pos  = line.find(str.encode('signal'))
            essid_pos   = line.find(str.encode('Beacon'))
            channel_pos = line.find(str.encode('CH:'))
            
            if (signal_pos != -1) and (essid_pos != -1) and (channel_pos != -1):            
                dB = line[:signal_pos].split()[-1]  # db is block before 'signal' [:-2] so dB is not printed
                essid = line[essid_pos:].split()[1][1:-1] # BSSID: begins with 'BSSID' [6:] so BSSID is not printed
                channel = line[channel_pos:].split()[1] #channel is the block before 'MHz'
                print(dB.decode(encoding) + ' ' +  essid.decode(encoding) + ' '+ channel.decode(encoding))
                #sys.stdout.flush()

    # Configure interface to monitor mode
    configureMonitorMode(interface)

    # Have network 
    interface = interface + 'mon'
    thread = threading.Thread(target = hooping, args=[interface, channels, interval])
    thread.daemon = True
    thread.start()
    
    # Receive packets 
    command = 'sudo -S tcpdump -npeqi {} -f type mgt subtype beacon'.format(interface)
    sniffing_process = runSudoCommand(command.split(), password)
    sniffing_output = sniffing_process.stdout

    # Extract RSSI and APs 
    thread = threading.Thread(target = filterOutput, args=[sniffing_output])
    thread.daemon = True
    thread.start()
