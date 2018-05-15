This folder contains the relevant schematic and files for the Spring 2018 Cuffless Blood Pressure Measurement Project

Contents:

	Powerpoints
		- Tutorial Presentation: Background information and initial project proposal
		- Update Presentations: Updates on progress through the semester
		- Final Presentation: Overall presentation of the project, work, and results

	Schematic and PCB Layout
		- CufflessPTT-v3.sch: The Eagle schematic for the PTT Measurement PCB 
		- CufflessPTT-v3.brd: The Eagle layout for the PTT Measurement PCB

	Software
		- PTT-BP.ino: The main Arduino code file for BP measurement.  
		- QRS_detection.ino: QRS detection algorithm for R wave detection
		- Si1143_pulse_demo.ino: Driver software for the Si1143, as well as helper functions for identification of the PPG signals
		- CUFFLESS.apk: Installer file for the Cuffless Android app.

	Solidworks Files
		- CAD files for the Cuffless case, as well as the locking pin to hold the case together.

Instructions for use:
	
	1. Install the Android app on an Android phone. 
	2. Press the button on the side to turn on the device. The LEDs should come on.
	3. Pair the Android app with the device by opening the app, and then find and pair to the device named HC-06.
	4. Press the start measurement button.
	5. Hold the electrode side of the device against the inside of your wrist. 
	6. Hold one finger of your opposite firmly on the electrdoe on top, and another finger firmly across the LEDs and photodiode.
	7. The device should display measurements as they are acquired.   