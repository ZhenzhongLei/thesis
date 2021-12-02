from wifi_rssi_collection.utils import *

def getAPinfo(output):
    """
    Extract AP info from network data 

    Args:
        output, string 

    Returns:
        A list of dictionary objects with 3 fields: "ssid", "quality" and "signal"
    """
    cells = output.split('Cell') # Divide raw string into raw cells.
    cells.pop(0) # Remove unneccesary "Scan Completed" message.
    if(len(cells) > 0): # Continue execution, if atleast one network is detected.
        def parseCell(cell):
            dictionary = {
                'ssid': getSSID(cell),
                'quality': getQuality(cell),
                'signal': getSignalLevel(cell)
            }
            return dictionary
        
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

        APinfo = [parseCell(cell) for cell in cells]
        return APinfo
    else:
        print("Networks not detected.")
        return None


def getRawNetworkScan(interface, password = 'luxc1', sudo=False):
    """
    Get raw network data by using iwlist command

    Args:
        interface: string, which is obtained by using command "iwconfig" from command line
        sudo: string, file extension, "txt" is given by default

    Returns:
        A dictionary with keys "output" and "error", which stores scanning result and error information respectively.
    """
    # Scan command 'iwlist interface scan' needs to be fed as an array.
    if sudo:
        command = ['sudo', '-S', 'iwlist', interface,'scan'] # -S will make Python read password from standard input
    else:
        command = ['iwlist', interface,'scan']
    # Open a subprocess running the scan command.
    process = Popen(command, stdin=PIPE, stdout=PIPE, stderr=PIPE)
    # Returns the 'success' and 'error' output.
    (output, error) = process.communicate(input=str.encode(password + '\n')) # input has to be byte type
    # Block all execution, until the scanning completes.
    process.wait()
    # Returns all output in a dictionary for easy retrieval.
    return {'output':output,'error':error}