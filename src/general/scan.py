from functools import reduce
from subprocess import Popen, PIPE
from general.utils import *

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
    runSudoCommand(command.split(), password).wait()
    command = "sudo -S wpa_cli -i {} scan_results".format(interface)
    process = runSudoCommand(command.split(), password)
    process.wait()
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
    if length > 1:
        for i in range(1, length): # Start from the second line of output
            fields = lines[i].split('\t')
            if len(fields) < 5:
                continue
            bssid = fields[0]
            signal = fields[2]
            ssid = fields[-1]
            frequency = fields[1]
            AP = {'bssid': bssid, 'signal': signal, 'ssid': ssid+ '(' + frequency + ')'}
            AP_info.append(AP)
    else:
        print("Networks not detected.")
    return AP_info

def runSudoCommand(command, password, input=PIPE, output=PIPE, err=PIPE, verbose=True):
    """
    Run sudo command by using Popen, be aware that IOError may occur. Use try-except block to handle this situation.

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
    process.stdin.write(str.encode(password + '\n')) # Method communicate will block the execution.
    return process