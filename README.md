# parking
Parkt das Auto ein
## Data channels
- MAVLINK_IN
- SENSORS
- CAR
## Config
```
<module>
            <name>parking</name>
            <channelMapping priority="20" from="CAR" to="CAR"/>
            <config>

                <!-- line fit auf den Punkten der Mittellinie -->
                <lineFitStartPoint>4</lineFitStartPoint>
                <lineFitEndPoint>7</lineFitEndPoint>

                <!-- Abmessungen einer validen Parklücke -->
                <minParkingSpaceSize>0.50</minParkingSpaceSize>
                <maxParkingSpaceSize>0.59</maxParkingSpaceSize>

                <!-- Geschwindigkeiten -->
                <velocitySearching>2.0</velocitySearching>
                <velocityEntering>0.8</velocityEntering>
                <velocityCorrecting>0.5</velocityCorrecting>
                <decelerationStopping>0.0</decelerationStopping>
                <velocityApproaching>0.8</velocityApproaching>

                <!-- ENTERING -->
                <xDistanceCorrection>-0.0</xDistanceCorrection> <!-- Korrektur für hohe Geschwindigkeiten beim Rückwärtsfahren -->
                <brakingDistanceUntilSteering>0.0</brakingDistanceUntilSteering>
                <alphaOffset>0.25</alphaOffset>
                <y0_worstCase>0.25</y0_worstCase>
                <k>0.02</k> <!-- safety distance zur Ecke der zweiten Box-->
                <d>0.125</d> <!-- safety distance zur ersten Box -->
                <maxSteeringAngle>24</maxSteeringAngle> <!-- in Grad-->

                <!-- CORRECTING -->
                <correctingDistances>0.06, 0.04, 0.03, 0.03, 0.02</correctingDistances>

                <!-- SEARCHING -->
                <medianFilterSize>5</medianFilterSize>
                <searchingPhiFactor>0.0</searchingPhiFactor> <!-- Korrekturwinkel falls das Auto schräg anfährt-->
                <lidarMeasurementsPerSecond>300</lidarMeasurementsPerSecond>
                <gradientThreshold>0.1</gradientThreshold> <!-- Für größere Werte wird eine Kante erkannt -->
                <maxDistanceLidar>0.75</maxDistanceLidar> <!-- Lidar cut-off distance -->
                <xMaxBeforeWorstCase>7</xMaxBeforeWorstCase>
                <!--minDistanceLidar>0.09</minDistanceLidar--> <!-- wurde wegen median filter entfernt -->

                <!-- WORST_CASE_BACKWARDS -->
                <xStartSearchingAgain>0.0</xStartSearchingAgain> <!-- Ab diesem x-Wert wird resettet und SEARCHING beginnt erneut -->
                <xMaxWorstCase>5.0</xMaxWorstCase>

                <!-- weitere Parameter -->
                <distanceMidLidarX>-0.03</distanceMidLidarX> <!-- Abstand Lidar zu Mitte Fahrzeug in x-Richtung -->
                <distanceMidLidarY>0.08</distanceMidLidarY> <!-- Abstand Lidar zu Mitte Fahrzeug in y-Richtung -->
                <minVelocityBeforeDrivingBackwards>0.3</minVelocityBeforeDrivingBackwards>
                <wheelbase>0.21</wheelbase>
                <carLength>0.36</carLength>
                <carWidth>0.25</carWidth>

                <!-- DEBUG -->
                <debugPrintEdges>false</debugPrintEdges>
            </config>
    </module>
```
## Dependencies
