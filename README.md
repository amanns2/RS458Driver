<h1>RS458Driver</h1>

A test setup to use the RS458 intervase as a network intervace on the device. 
This was a test to see to see how a The Network subsystem and the serial device drive can be combinded together. 

How to build the source code:
<ul>
  <li> Install all the SDK for the YOKTO build. </li>
  <li> Adjust the Device-Tree config files and update the Kernal. </li>
  <li> Run the build.sh file to build the Makefile. </li>
  <li> Load the Kernel module on the target device. </li>
  <li> A new network interface should pop up. </li>
</ul>


Known Bugs:
<ul>
  <li> So far only RS232 is supported nd not RS458. This leads to problems with the flow controle. </li>
  <li> sSometime the connection gets really slow</li>
</ul>
