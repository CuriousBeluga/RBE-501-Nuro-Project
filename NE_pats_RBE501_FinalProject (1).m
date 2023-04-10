%% RBE 501: Path Planning Nuro Project
% Sean Tseng, Debbie G, Lucas Vanslette
%% Initialize Occupancy Map
% This section loads the road map that the robot will be navigating in this 
% project. The first step is to binarize the road map imagine. Road map is still 
% in progress, replace the name in 'imread' to read in a different image for path 
% planning.
% 
% Dubins explanation: <https://demonstrations.wolfram.com/IsochronsForADubinsCar/ 
% https://demonstrations.wolfram.com/IsochronsForADubinsCar/>

clear
clc
resize_fac =10;
J = rgb2gray(imread('City map.png'));

% (do this for the WPI map)
%J(J < 240) = 0;
%J(J >= 240) = 255;
%J = J(:,:,1);
%map = binaryOccupancyMap(J,resize_fac);

% (do this for test map)
J(J > 50) = 255;
map = binaryOccupancyMap(~J,resize_fac);


%% Initialize Simulation Parameters
% Update state space bounds to be the same as map limits, which were generated 
% earlier with the binaryOccupancyMap method. Also, create a Dubins model of the 
% vehicle with bounds imported from the occupancy map.
bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.00000005;

%% 
% Create a state validator using the created state space and based on the occupancy 
% map created from the png image.
sv = validatorOccupancyMap(ss);
sv.Map = map;

%% 
% Set validation distance for the simulation.
sv.ValidationDistance = 0.02;

%% 
% Now the planner can be created with a modifiable maximum distance between 
% nodes of the tree.
Maxdistance = 0.4;

%% 
% Enter the weight of the groceries, in pounds

% average household grocery order (assumes the same for each, assumes waypoint order size is smaller than max capacity)
% average grocery order = 75$
% 2019 average retail price per pound = 2.31$
%   https://refed.org/downloads/ReFED-U.S.-Grocery-Retail-Value-to-Weight-Conversion-Factors.pdf
% 75/2.31 = 32.46 average pounds per grocery order
order_size = 32.46;     % lbs

% Nuro can carry 500 pounds of payload
nuroCapacity = 500;     % lbs
% dodge ram can carry 2300 pounds of payload
ramCapacity = 2300;     % lbs


%% 
% Calculate the random waypoints that the car will traverse
plotting = 0;   % 0 = false, 1 = true

% calculate how many times we want to run our simulation
num_sims = 10;

num_stops_vec = zeros(num_sims,1);

num_waypoints = zeros(2,num_sims);

% storage for emissions data
plotNuroEmiss = zeros(1,num_sims);
plotRamEmiss = zeros(1,num_sims);


for i=1:num_sims
    message = append('Simulation number: ',num2str(i));
    disp(message)

    num_stops = i*10;
    num_stops_vec(i) = num_stops;

    stops = get_random_stops(map, num_stops);
    
    % waypoints differ based on total number of stops
    nuroWaypoints = calculate_waypoints(map, stops, num_stops, nuroCapacity, order_size, plotting);
    ramWaypoints = calculate_waypoints(map, stops, num_stops, ramCapacity, order_size, plotting);

    % store number of waypoints created for each
    num_waypoints(1,i) = length(nuroWaypoints);
    num_waypoints(2,i) = length(ramWaypoints);

    %% 
    % Determine which path planning algorithm will be used for simulation
    % 1: standard RRT
    % 2: RRT*
    % 3: Bi-RRT
    algorithm = 3;

    %% 
    % *Start simulation*
    distance =0;
    % added function that runs below simulation X times
    carType = 'Nuro';
    plotNuroEmiss(1,i) = simulation(map, algorithm, nuroWaypoints, ss, sv, Maxdistance, ...
                         carType, ~plotting);
    carType = 'RAM';
    plotRamEmiss(1,i)= simulation(map, algorithm, ramWaypoints, ss, sv, Maxdistance ...
                        ,carType, plotting);
end


% plot carbon emission from both cars per number of total delivery stops
plot(num_stops_vec, plotRamEmiss)
hold on
plot(num_stops_vec, plotNuroEmiss)
title('Emissions vs Number of Deliveries')
ylabel('Carbon Emissions')
xlabel('Number of Deliveries')
legend('Ram', 'Nuro')
hold off

% plot number of waypoints made from both cars per number of total delivery stops
plot(num_stops_vec, num_waypoints(2,:))
hold on
plot(num_stops_vec, num_waypoints(1,:))
title('Total Waypoints vs Number of Deliveries')
xlabel('Number of Deliveries')
ylabel('Total Waypoints')
legend('Ram', 'Nuro')
hold off

%% 
function stops = get_random_stops(map, num_deliveries)
    % waypoints = [  x1     x2   ...
    %                y1     y2   ...
    %              theta1 theta2 ... ]
    % % % test case
    % waypoints = [91  84 60;   %x
    %              44  20 24;   %y
    %              4.5 1  1 ; ] %theta

    % stops includes all delivery locations plus the store location
    stops = zeros(3,num_deliveries+1);
    % for each delivery location
    for i=1:num_deliveries+1
        % generate random orientation
        rand_theta = -pi + (2*pi)*rand;
        % determine if random coordinate is valid
        while 1
            rand_x = map.XWorldLimits(2)*rand;
            rand_y = map.YWorldLimits(2)*rand;
            if ~checkOccupancy(map, [rand_x rand_y])
                break
            end
        end
        % save as a new waypoint
        stops(:,i) = [rand_x; rand_y; rand_theta;];
    end
end

%% *Function calculate_waypoints: given car, calculate all places that the car 
% will stop on its route*
function waypoints = calculate_waypoints(map, stops, num_deliveries, carCapacity, order_size, plotting)
    % remove initial waypoint as a separate stop from the rest of the
    % waypoints
    % example: waypoints should have 4 locations, init_wpt should be the
    %           last of the 5 locations created
    init_wpt = stops(:,end);
    waypoints = stops(:,1:end-1);

    % added number of times to refill, which is the minimum number of times Nuro would have
    %   to go back to the start and refill its capacity for groceries
    % example: carCapacity = 10, order_size = 5, so you would need to go
    %           back every 2 trips
    group_size = floor(carCapacity / order_size);

    % group each stop into a group of size num_stops_until_refill
    num_groups = ceil(num_deliveries/group_size);
    wpt_groups = cell(num_groups, group_size+1);
    wpt_index = 1;
    for i=1:num_groups
        for j=1:group_size
            % if we have more stops available than total deliveries, don't
            % add any more deliveries after the ones in the waypoints list
            % have been added
            if and(j <= num_deliveries, wpt_index <= length(waypoints)) 
                wpt_groups{i,j} = waypoints(:,wpt_index);
                wpt_index = wpt_index+1;
            end
        end
    end
    % example: num_groups should now have 2 cell arrays, each of which
    %           containing two of the four waypoints

    % append the starting waypoint to the end of each of the groups
    for i=1:num_groups
        wpt_groups{i,group_size+1} = init_wpt;
    end

    % append each waypoint group together
    new_waypoints = zeros(3,group_size+1);
    wpt_index = 1;
    for i=1:num_groups
        for j=1:group_size+1
            if wpt_groups{i,j} ~= 0
                new_waypoints(:,wpt_index) = wpt_groups{i,j};
                wpt_index = wpt_index + 1;
            end
        end
    end

    % remove waypoints that are [0;0;0]
    new_waypoints(:,wpt_index:end) = [];

    % add starting waypoint to the start of the final matrix
    waypoints = [init_wpt new_waypoints];

    if plotting
        show(map)
        if carCapacity == 500
            carType = "Nuro";
        else
            carType = "RAM";
        end
        title(append("Total Waypoints for ",carType));
        set(gca,'XTick',[], 'YTick', [])
        xlabel('');
        ylabel('');
        hold on
        % plot waypoints in BIG lettering
        % scatter(x,y,sz,symbol,'filled')
        scatter(waypoints(1, 2:end-1), waypoints(2, 2:end-1), 140, 'filled');
        % plot the first waypoint (the store) in a different way
        scatter(waypoints(1,1), waypoints(2,1), 140, 'r', 'filled');
        legend('Goal Locations', 'Store', 'Location', 'northoutside');
        hold off
    end
end


%%
% -------------------------------------------------------------------------------------

function [emiss]=simulation(map, algorithm, waypoints, ss, ...
            sv, Maxdistance,carType,plotting)
%% User Defined Path Planning
% Choose what planning algorithm to use, options include RRT, RRT*, Bidirectional 
% RRT, Hybrid A*, A* from the drop down menu. The A* algorithm only accepts a 
% 2x1 vector instead of a 3x1 vector like the other algorithms.

choose = algorithm;
    switch choose
        case 1
            planner = plannerRRT(ss,sv,MaxConnectionDistance=Maxdistance);
        case 2
            planner = plannerRRTStar(ss,sv,MaxConnectionDistance=Maxdistance);
        case 3
            planner = plannerBiRRT(ss,sv,MaxConnectionDistance=Maxdistance);
    end
planner.MaxIterations = 10000000;
%% Plotting Results
% If the user wants to select the option to see the whole tree expansion or 
% not in the dropdown below.

%plot_choose = false;
%% 
% Enter the number of waypoints for the simulation

% changed to add waypoints based on number of inputted waypoints
wayP = length(waypoints);
distance = zeros(1,wayP);
% RRT and RRT* Plot
% This else condition is for the RRT and RRT* algorithms, the plan function 
% creates plan based on the  arguments inputed into the function. Most of the 
% path planning algorithms are plotted the same way, but the RRT algorithms are 
% more complex than that of the A* derivatives.
% 
% All path planning algorithms must create a plan using the plan function, which 
% takes the planner object created in the *'User Defined Path Planning Section'*. 
% This returns a navPath object and a data structure called 'solnInfo' used for 
% plotting.

if choose == 1 || choose == 2
%% 
% The results are then collected and presented in graphical form. For RRT based 
% algorithms, the tree must iterated over and each node fed into the plot graph 
% from the data structure object called solnInfo. Then the path is drawn using 
% data from the navPath object.
if plotting
    show(map)
        % Must always change the title in this section here
    string_title = append("Planned Path",carType);
    title(string_title)
    set(gca,'XTick',[], 'YTick', [])
    xlabel('');
    ylabel('');    
    grid on
    hold on
end
%% 
% The for loop is for iterating through the entire set of waypoints that the 
% user has given.

legend_plots = cell(wayP-1,1);

for i =1:wayP-1
    start = waypoints(:,i)';
    goal = waypoints(:,i+1)';

    if plotting
        [pthObj,~] = plan(planner,start,goal);
    end
    path=plan(planner,start,goal);
    % if the path returns with no path, run it again. there's a valid path
    % somewhere.
    while path.NumStates == 0
        path=plan(planner,start,goal);
    end

%% 
% The path length is found using the function pathLength, this will be used 
% to determine the emission of the vehicle's path.

    distance(1,i)= pathLength(path);
%% 
% If the user wants to plot the whole tree expansion, the if condition will 
% read true and the code to plot the expansion will execute, otherwise only the 
% planned path will be plotted.

    % if plot_choose ==true
    %     % Tree expansion
    %     plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
    % end

    % Draw path for this waypoint path
    
    if plotting
        plot(pthObj.States(:,1),pthObj.States(:,2),'-','LineWidth',2)
        legend_plots{i} = sprintf('Waypoint %i to %i', i, i+1);
    end
end



% Bidirectional RRT Plot
% <https://ieeexplore.ieee.org/document/9638379 https://ieeexplore.ieee.org/document/9638379>
% 
% This section is for plotting the results from Bidirectional RRT algorithm. 
% The code is generally the same as that of RRT and RRT*, but there are 2 tree 
% expansions, one for the forward tree, and one for the backwards tree. When these 
% two trees meet each other, the backward tree will act as a heuristic to guide 
% the forward tree to continuously grow toward the goal state, where the algorithm 
% switches to unidirectional search mode. 

elseif choose == 3
%% 
% There is also an option here to enable connect heuristic for the planner object.
if plotting
    show(map)
    string_title = append("Bi-RRT Planned Path ",carType);
    title(string_title)
    set(gca,'XTick',[], 'YTick', []);
    xlabel('');
    ylabel('');
    hold on
end

planner.EnableConnectHeuristic = false;

legend_plots = cell(wayP-1,1);

% for each waypoint
for i =1:wayP-1
    start = waypoints(:,i)';
    goal = waypoints(:,i+1)';

    if plotting
        [pthObj,~] = plan(planner,start,goal);
    end
    path =  plan(planner,start,goal);
    distance(1,i)= pathLength(path);
    
%% 
% Now the tree expansion can be drawn, if the user selected to show all tree 
% expansions, the if condition will be read as true, otherwise only the optimized 
% path will be drawn.

    % Start tree expansion
    % if plot_choose == true
    % plot(solnInfo.StartTreeData(:,1),solnInfo.StartTreeData(:,2), ...
    %     '.-','color','b')
    % % Goal tree expansion
    % plot(solnInfo.GoalTreeData(:,1),solnInfo.GoalTreeData(:,2), ...
    %     '.-','color','g')
    % end

    % Draw path
    if plotting
        plot(pthObj.States(:,1),pthObj.States(:,2),'-','LineWidth',2);
        legend_plots{i} = sprintf('Waypoint %i to %i', i, i+1);
    end
end
%% 
% end Bi-RRT planning algorithm

end
%% 
% 

if plotting
    % plot waypoints in BIG lettering
    % scatter(x,y,sz,symbol,'filled')
    scatter(waypoints(1, 2:end), waypoints(2, 2:end), 140, 'filled');
    % plot the first waypoint (the store) in a different way
    scatter(waypoints(1,1), waypoints(2,1), 140, 'r', 'filled');
    legend_plots{end+1} = 'Goal Locations';
    legend_plots{end+1} = 'Store';
    legend(legend_plots, 'Location', 'northoutside');
    hold off
end
%% Calculating Vehicle Emissions
% Nuro Emissions Calculations

if carType == "Nuro"
%% 
% This section will be for calculating the emission of the vehicle. The Nuro 
% autonomous vehicle has the following stats:
% 
% source: <https://www.sciencedirect.com/science/article/pii/S1361920920306301?casa_token=4RF5gYc7sf0AAAAA:7oqllUDzDqQI1obliIZy6W-ZWc2Pb8iyKLOGftvfJhuyTarsgXleM2OWCz4InrdWO_vEl0Nyhw 
% https://www.sciencedirect.com/science/article/pii/S1361920920306301?casa_token=4RF5gYc7sf0AAAAA:7oqllUDzDqQI1obliIZy6W-ZWc2Pb8iyKLOGftvfJhuyTarsgXleM2OWCz4InrdWO_vEl0Nyhw>
%% 
% * Payload = 110 kg
% * range = 16.1 km
% * Energy consumption = 139.6 wh/km
%% 
% Factoring in energy production for the state of MA:
% 
% <https://www.eia.gov/state/?sid=MA#tabs-4 https://www.eia.gov/state/?sid=MA#tabs-4>
% 
% Dec 2022:
%% 
% * Petroleum-Fired = 8.9%, 180,000 MWh
% * Natural Gas Fired = 73.9%  1,492,000 MWh
% * Coal Fired = 0.0%
% * Nuclear = 0%
% * Renewables = 15% 303,000 MWh
%% 
% <https://www.forestresearch.gov.uk/tools-and-resources/fthr/biomass-energy-resources/reference-biomass/facts-figures/carbon-emissions-of-different-fuels/ 
% https://www.forestresearch.gov.uk/tools-and-resources/fthr/biomass-energy-resources/reference-biomass/facts-figures/carbon-emissions-of-different-fuels/>
% 
% Assessing and averaging total carbon emission of MA. Natural gas is 202 kg/Mwh, 
% Petroleum is 260 kg/MWh. Therefore MA carbon emission stands at 348,184,000 
% kg CO_2 in December.
% 
% 

%Find weighted average 
p_energy = 180000;
gas_energy = 1492000;
renewables = 303000;    
carbon_energy = p_energy + gas_energy + renewables;
p_weight = p_energy/carbon_energy;
gas_weight = gas_energy/carbon_energy;
%% 
% This weighted average calculates the actual MA carbon emission as kg CO2 per 
% MWh of carbon based fuel burned. 

%Employ formula
p_footprint = 260;
gas_footprint = 202;
avg_MA_carbon= p_weight*p_footprint+gas_weight*gas_footprint;
%% 
% Now using information from Nuro, convert the kg/MWh of the MA electric grid 
% to kg/wh and then multiply that by the Nuro's wh/km to get the average kg CO2 
% per km of the Nuro vehicle, which is the following:

nuro_carbon = avg_MA_carbon/1000000*139.6;
%% 
% Now the total distance traveled will be multiplied by Nuro's carbon footprint 
% per kilometer to yield the total emissions for the trip.

%calculating emissions (just placeholder sample) change the the calculation
%to actual emissions data
total_distance = sum(distance,'all')*.1; %to get km from hectometer --> need to change  
disp(append(num2str(total_distance),' km traveled'));
emiss = total_distance*nuro_carbon;
disp(append('Total emissions from Nuro (kilograms carbon): ',num2str(emiss)));

%% Dodge RAM Emissions
% This is section is for calculating the emissions from the dodge ram vehicle 
% if it is chosen as the vehicle
% 
% <https://www.fueleconomy.gov/feg/bymake/Ram2021.shtml https://www.fueleconomy.gov/feg/bymake/Ram2021.shtml>
% 
% 2021 RAM Promaster City has 24 MPG, tank size 16.1 gallons, 1760 pounds payload, 
% about 800 kg 
% 
% <https://www.eia.gov/environment/emissions/co2_vol_mass.php https://www.eia.gov/environment/emissions/co2_vol_mass.php>
% 
% Gasoline emits about 8.78 kg CO2 per gallon

else
   %mpg = 24;
   kpg = 38.62;
   tank_size = 16.1; %gallons
   RAM_tank_emiss=tank_size*8.78; %emissions per full tank
   range = tank_size*kpg; %miles per full tank
   RAM_carbon_mile = RAM_tank_emiss/range;
   total_distance = sum(distance,'all')*.1; %to get km from hectometer --> need to change  
   disp(append(num2str(total_distance),' km traveled'))
   emiss = total_distance*RAM_carbon_mile;
   disp(append('Total emissions from RAM simulation (kilograms carbon): ',num2str(emiss)));
end


end