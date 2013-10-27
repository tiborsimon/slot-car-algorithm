//==============================================================================
//------------------------------------------------------------------------------
//       S L O T   C A R   P E R I O D   F I N D E R   A L G O R I T H M
//------------------------------------------------------------------------------
//==============================================================================
//
// This SciNote demonstrates the operation of a very simple thus very specific
// period finder algorithm, that was developed for a autonome real-time slot
// car race.
//
//==============================================================================
//
// Copyright (c) 2013 Tibor Simon
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
//==============================================================================
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
//==============================================================================

clear
clc
close

//==============================================================================
//  P L O T  A C C E L E R A T I O N   D A T A 
//==============================================================================
data = csvRead("thu.csv",";");  // thu.csv, because it was recorded on thursday

time = data(:,1);
accX = data(:,2);
accY = data(:,3);
accZ = data(:,4);
trackVoltage = data(:,5);
motorCurrent = data(:,6);

subplot(311)
plot(accX)
title("Acceleration X")
subplot(312)
plot(accY)
title("Acceleration Y")
subplot(313)
plot(accZ)
title("Acceleration Z")



//==============================================================================
//------------------------------------------------------------------------------
//                P E R I O D   F I N D E R   A L G O R I T H M
//------------------------------------------------------------------------------
//==============================================================================



//==============================================================================
//  I N P U T   D A T A
//==============================================================================
// The input data, which the algorithm operates.
data = accX;

//==============================================================================
//  S I G N A L S   F O R   E V E N T   O B S E R V A T I O N
//==============================================================================
// To demonstrate the algorithm we need some visualization techniques. I choose
// additional signals to visualise events.
SIGNAL_car_state = zeros(size(data));
SIGNAL_start_state_2 = zeros(size(data));
SIGNAL_enough_data = zeros(size(data));
SIGNAL_periodicity = zeros(size(data));



//==============================================================================
//  A D J U S T A B L E   P A R A M E T E R S
//==============================================================================
THRESHOLD_UP            = 33000; // we are sensing cornering above this threshold
THRESHOLD_DOWN          = 31000; // we are sensing cornering below this threshold
SPEED_UP_DELAY          = 150; // speed up counter
EVENTS_LENGTH_MAX_DIFF  = 12; // max difference in samples between identical events


//==============================================================================
//  A L G O R I T H M   S T A T E S
//==============================================================================
//  1 - Waiting for the car to speed up to learning speed. After it is speeded
//      up, the algorithm waits for reaching a straight line of track. The 
//      period finding won't start while the car is moving in a corner.
//  2 - Looking for the period. 
//  3 - Futási állapot. A perióduskeresési adatok valamint a megadott
//      nemlineáris paraméterek és az idömérés alapján gyorsít vagy lassít.
//      Három féle sebességi érték van. Üresjárat (motorVoltage=0),
//      maximális kicsúszásmentes sebesség, maximális sebesség (motorVoltage
//      = 6000)
STATE_SPEEDING_UP       = 1;
STATE_PERIOD_FINDING    = 2;
STATE_RUN               = 3;

state = STATE_SPEEDING_UP;

//==============================================================================
//  G L O B A L   C O U N T E R 
//==============================================================================
// On the car there is no for loop that provides the index variable. We have to
// do it manually.
globalCounter = 0;


//==============================================================================
//  C A R   S T A T E S
//==============================================================================
// The state of the car calculated by the current acceleration values
//   0 - lower corner
//   1 - straight
//   2 - upper corner
CAR_STATE_LOWER_CORNER  = 0;
CAR_STATE_STRAIGHT      = 1;
CAR_STATE_UPPER_CORNER  = 2;

carState = 0;

// Last car state for edge detection
lastCarState = 0;

//==============================================================================
//  E V E N T   V A R I A B L E S
//==============================================================================
// To find the period of the signal, we have to count the events. For the clear 
// decision, we must have at least two of all events.
downCounter = 0;
straightCounter = 0;
upCounter = 0;

// To calculate the lenght of the events we have to have a reference pointer,
// that stores the last event length.
lastEventTime = 0;

// Structure to store the events.
events(1).type = 0;   // type
events(1).length = 0; // length

// Counter for the events
eventCounter = 0;


//==============================================================================
//  R E S U L T   V A R I A B L E S
//==============================================================================
// Index of the calculated period
periodCounter = 0;  // to drive the car
period = 0; // to hold the periodicity




//==============================================================================
//------------------------------------------------------------------------------
//                      P R O C E S S I N G   L O O P
//------------------------------------------------------------------------------
//==============================================================================


for i=1:length(data)

    // Increasing the global counter
    globalCounter = globalCounter + 1;
    
    
    //==========================================================================
    //  C A L C U L A T E   C U R R E N T   C A R   S T A T E
    //==========================================================================
    // Save previous state
    lastCarState = carState;
    
    // Calculate new state and 
    if data(i) > THRESHOLD_UP then
        // Greater than the upper threshold
        SIGNAL_car_state(i) = THRESHOLD_UP+3000;
        carState = CAR_STATE_UPPER_CORNER;
        
    elseif data(i) < THRESHOLD_DOWN then
        // Lower than the lower threshold
        if data(i) ~= 0 then
            SIGNAL_car_state(i) = THRESHOLD_DOWN-3000;
            carState = CAR_STATE_LOWER_CORNER;
        else
            // The accelerometer can provide faulty values that we need to
            // eliminate. Ignore this samples.
            if i>1 then
                SIGNAL_car_state(i) = SIGNAL_car_state(i-1);
            end
        end
        
    else
        // Between the two thresholds
        SIGNAL_car_state(i) = 32000;
        carState = CAR_STATE_STRAIGHT;
        
    end
    
    // Ignoring the faulty 
    if data(i) ~= 0 then
        
        //======================================================================
        //----------------------------------------------------------------------
        //        P E R I O D   F I N D E R   S T A T E   M A C H I N E 
        //----------------------------------------------------------------------
        //======================================================================
        
        //======================================================================
        //  F I R S T   S T A T E
        //======================================================================
        
        if state == STATE_SPEEDING_UP then
            
            // Waiting for speeing up
            if (globalCounter > SPEED_UP_DELAY) then
                // The delay has ended. Let's wayt until the car is running on
                // a straight section.
                if carState == CAR_STATE_STRAIGHT then
                   state = STATE_PERIOD_FINDING;  // switch state
                   disp('START');
                   SIGNAL_start_state_2(i) = 40000; // green signal
                   lastEventTime = 0;
                end
            end
            
            
        //======================================================================
        //  S E C O N D   S T A T E
        //======================================================================
        
        elseif state == STATE_PERIOD_FINDING then
        
            // Checking for changing in car state.
            if lastCarState ~= carState then
                
                
                
                
                //==============================================================
                //  P R E P A R I N G   E V E N T S
                //==============================================================
                // Collect samples until we have at least two of all kind of
                // events.
                
                eventCounter = eventCounter + 1; // increase the event index
                    
                // Store the last event. At this point we know that the current
                // state is ended, so we store it nad we calculate its length.
                events(eventCounter).type = lastCarState;
                events(eventCounter).length = globalCounter - lastEventTime;
                    
                // Set the beginning of the new event
                lastEventTime = globalCounter;
                    
                // Counting the events
                if lastCarState == CAR_STATE_LOWER_CORNER then
                    downCounter = downCounter + 1;
                elseif  lastCarState == CAR_STATE_STRAIGHT then
                    straightCounter = straightCounter + 1;
                elseif lastCarState == CAR_STATE_UPPER_CORNER then
                    upCounter = upCounter + 1;
                end
                
                
                //==============================================================
                //  P E R I O D   F I N D E R   A L G O R I T H M
                //==============================================================
                // Minimal number of data has collected. Run the period finder 
                // algorithm.
                if (downCounter >= 2) & (straightCounter >= 2) & (upCounter >= 2) then
                    
                    SIGNAL_enough_data(i) = 40000;
                    
                    // Let's look for similar events in the list like the last one!
                    // Check the event type then the event lenght with a small error margin.
                    for k = (eventCounter-1):-1:1
                        if (events(k).type == events(eventCounter).type) then
                            if (abs(events(k).length - events(eventCounter).length)) < EVENTS_LENGTH_MAX_DIFF then
                                // If one previous event match the last one, check the event before the matching
                                // to make sure this match comes from the periodicity
                                if events(k-1).type == events(eventCounter-1).type then
                                    if abs(events(k-1).length - events(eventCounter-1).length) < EVENTS_LENGTH_MAX_DIFF then
                                        // If everithing match, we have found the periodicity of the track.
                                        state = STATE_RUN;
                                        disp('Period has found!');
                                        
                                        // Calculate the periodicity information
                                        period = eventCounter - k;
                                        periodIndexBase = k+1;
                                        SIGNAL_periodicity(i) = 40000;
                                    end
                                end
                            end
                        end
                    end // forciklus vége
                end // kellö számú esemény ellenörzésének vége
            end
        
        
        //======================================================================
        //  T H I R D   S T A T E
        //======================================================================
        
        elseif state == STATE_RUN then
            
            // Driving the car based on the gained information.
            if lastCarState ~= carState
                
                periodCounter = periodCounter + 1;
                if periodCounter >= period
                   periodCounter = 0;
                   SIGNAL_periodicity(i) = 40000;
                end
                
            end
            
        end
    end
end
    


//==============================================================================
//  P L O T   T H E   R E S U L T S 
//==============================================================================
 
scf();
plot(data,'b');
plot(SIGNAL_car_state,'r','linewidth',2);
plot(SIGNAL_start_state_2,'g','linewidth',2);
plot(SIGNAL_enough_data,'m','linewidth',2);

plot(SIGNAL_periodicity,'ko');

legend('Acceleration data',...
'Car state', ...
'State 2 starts',...
'Enough data to search',...
'Online calculated periodicity',...
'legend_location','upper_caption');
set(gca(),"data_bounds",matrix([0,1200,15000,45000],4,-1)); 



