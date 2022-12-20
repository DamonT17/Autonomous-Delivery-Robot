package my.ADR;

import gnu.io.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;

import static my.ADR.LoadItem.username;
import static my.ADR.LoadItem.password;
import static my.ADR.LoadItem.url;

public class SerialIO implements SerialPortEventListener 
{
    SerialPort serialPort;

    private static final int TIME_OUT = 250;
    private static final int DATA_RATE = 115200;
    
    private BufferedReader input;
    private OutputStream output;

    private final Object stopRead = new Object();
    private final Object stopWrite = new Object();
    public boolean stopR = false;
    public boolean stopW = false;

    String inputLine;
    String hexCheck = "-?[0-9A-F]+";
    String alphaCheck = "-?[A-F]+";
    String numCheck = "-?[0-9]+";
    
    private int[] nib = new int[4];
    public int[] inputVal = new int[2];
    public int byteVal;
    
    public static String cfmMessage;
    public static final String opPhone = "19253219589";
    
    public void initialize()
    {       
        String portID = null;
        Enumeration portEnum = CommPortIdentifier.getPortIdentifiers();
        
        while(portEnum.hasMoreElements())
        {
            CommPortIdentifier portIdentifier = (CommPortIdentifier) portEnum.nextElement();
            if(!portIdentifier.isCurrentlyOwned() && portIdentifier.getPortType() == CommPortIdentifier.PORT_SERIAL)
            {
                portID = portIdentifier.getName();
                System.out.println(portID);
                break;
            }  
        }
        
        if(portID == null)
        {
            System.out.println("Could not find COM port.");
            return;
        }
        
        try 
        {           
            CommPortIdentifier currPort = CommPortIdentifier.getPortIdentifier(portID);
            serialPort = (SerialPort) currPort.open(this.getClass().getName(), 
                        TIME_OUT);
            serialPort.setSerialPortParams(DATA_RATE, 
                        SerialPort.DATABITS_8, 
                        SerialPort.STOPBITS_1,
                        SerialPort.PARITY_NONE);
            
            input = new BufferedReader(new InputStreamReader(serialPort.getInputStream(), StandardCharsets.US_ASCII));
            output = serialPort.getOutputStream();
            
            serialPort.addEventListener(this);
            serialPort.notifyOnDataAvailable(true);
        } catch (Exception e) {System.err.println(e.toString());}
    }
    
    public synchronized void close() 
    {
        if (serialPort != null) 
        {
            serialPort.removeEventListener();
            serialPort.close();
        }
    }
    
    @Override
    public synchronized void serialEvent(SerialPortEvent s_evt)
    {
        if(s_evt.getEventType() == SerialPortEvent.DATA_AVAILABLE)
        {	    
            try
            {
                inputLine = input.readLine();
                inputToByte();

                if((MainFrame.deliveryPanel.isVisible()) && (inputVal[0] == MainFrame.loadPanel.roomVal[0]))
                {
                    MainFrame.deliveryPanel.setVisible(false);
                    MainFrame.arrivalPanel.setVisible(true);
                    
                    cfmMessage = "ADR has arrived safely at Room " + MainFrame.entryPanel.room;
                    //MainFrame.sms.SendSMS(username, password, cfmMessage, opPhone, url);
                }
                
                if((MainFrame.returnPanel.isVisible()) && ((inputVal[0] == MainFrame.ratingPanel.homeVal[0]) || (inputVal[0] << 4) == MainFrame.ratingPanel.homeVal[0]))
                {
                    MainFrame.entryPanel.entryCount = 0;
                    MainFrame.entryPanel.chCount = 0;

                    /**** Entry Panel Reset ****/
                    Arrays.fill(MainFrame.entryPanel.phoneNumber, 0, 10, '\0');
                    Arrays.fill(MainFrame.entryPanel.roomNumber, 0, 4, '\0');
                    MainFrame.entryPanel.phone = "";
                    MainFrame.entryPanel.room = "";

                    MainFrame.entryPanel.answrLabel.setText("");
                    MainFrame.entryPanel.enter.setEnabled(false);
                    MainFrame.entryPanel.delete.setEnabled(false);

                    MainFrame.entryPanel.msgLabel.setText("<html><center>Please enter your phone number:</center></html>");   // Reset
                    MainFrame.entryPanel.back.setText("cancel");     // Reset
                    MainFrame.entryPanel.cancel.setVisible(false);   // Reset
                    MainFrame.entryPanel.cancel.setEnabled(false);   // Reset

                    /**** LoadItem Panel Reset ****/
                    MainFrame.loadPanel.pcHolder.delete(0, 4);
                    Arrays.fill(MainFrame.loadPanel.passCode, 0, 4, '\0');
                    MainFrame.loadPanel.pc = "";

                    /**** Pass code Panel Reset ****/
                    MainFrame.passcodePanel.pcCount = 0;

                    Arrays.fill(MainFrame.passcodePanel.userPC, 0, 4, '\0');
                    MainFrame.passcodePanel.userPasscode = "";

                    MainFrame.passcodePanel.answrLabel.setText("");
                    MainFrame.passcodePanel.enter.setEnabled(false);
                    MainFrame.passcodePanel.delete.setEnabled(false);

                    /**** Arrival Panel Reset ****/
                    MainFrame.arrivalPanel.items.setText("Retrieve items");

                    MainFrame.returnPanel.setVisible(false);
                    MainFrame.startPanel.setVisible(true);
                    
                    /**** Rating Panel Reset ****/
                    MainFrame.ratingPanel.homeVal[0] = 0x00;
                }
            } 
            catch(IOException e) {System.err.println(s_evt.toString());}
	}
    }  
    
    public void inputToByte()
    {
        String[] str = new String[4];
        
        if(inputLine.length() == 2)
        {
            inputLine = '0' + inputLine.substring(0, 1) + '0' + inputLine.substring(1, 2);
        }
        else if(inputLine.length() == 3)
        {
            inputLine = '0' + inputLine;
        }
        
        if(inputLine.length() == 4 && inputLine.matches(hexCheck))
        {
            for(int i = 0; i < inputLine.length(); i++)
            {
                str[i] = inputLine.substring(i,i+1);
                       
                if(str[i].matches(numCheck))
                    nib[i] = str[i].charAt(0) - '0';
                else if(str[i].matches(alphaCheck))
                    nib[i] = str[i].charAt(0) - '7';
            }
            
            byteVal = (((nib[0] << 4) | nib[1]) << 8) | ((nib[2] << 4) | nib[3]);
            
            inputVal[0] = (nib[0] << 4) | nib[1];   // Room #
            inputVal[1] = (nib[2] << 4) | nib[3];   // Flags
        }
    }

    public void transmit(byte[] b)
    {
	try
	{
	    output.write(b);
	    synchronized(stopWrite) {stopW = false;}
	}
	catch(IOException e) {}
    }
}
