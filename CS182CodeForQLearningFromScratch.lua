-------- HELPER FUNCTIONS FOR SIMULATION --------------


-- Vector functions
function addVectors(x,y)
    return {x[1] + y[1],x[2] + y[2],0}
end

function subtractVectors(x,y)
    return {x[1] - y[1],x[2] - y[2],0}
end

function multiplyVector(vec,c)
    return {vec[1]*c, vec[2]*c, 0}
end

function printVector(o)
    print("x: ".. o[1],"y: ".. o[2],"z: ".. o[3])
end

function printTable(x)
    -- prints table of 3D-vectors
    n = table.getn(x)
    for i = 1,n,1 do 
       printVector(x[i])
    end
end

function printTable2(x)
    -- prints table of scalars
    n = table.getn(x)
    for i = 1,n,1 do 
       print(x[i])
    end
end

function getUnitVector(x)
    mag = (x[1]^2.0 +  x[2]^2.0)^(.5)
    return {x[1]/mag, x[2]/mag, 0}
end

function getMagVector(x)
    return (x[1]^2.0 +  x[2]^2.0)^(.5)
end

function getPerpVector(x)
    return {-x[2], x[1], 0}
end


-- moves leg to new relative position by moving foot up halfway then placing it down
-- this version of the function moves the leg relative to the robot's frame 
-- axis relative to joint 4 and 1
function moveLegRel(whichTarget, liftHeight, relPosX, relPosY)

    old_pos = simGetObjectPosition(whichTarget, -1)


    right_vertex = simGetObjectPosition(jointTips[right_v_num], -1)
    left_vertex = simGetObjectPosition(jointTips[left_v_num], -1)
    axis_vector = subtractVectors(right_vertex, left_vertex)
    unit_vector = getUnitVector(axis_vector)
    perp_vector = getPerpVector(unit_vector)
    
    
    relPosnew = addVectors(multiplyVector(unit_vector,relPosX), multiplyVector(perp_vector,relPosY))

    new_pos = {0,0,0}
    new_pos[1] = old_pos[1] + relPosnew[1]/2.0
    new_pos[2] = old_pos[2] + relPosnew[2]/2.0
    new_pos[3] = liftHeight
    
    simMoveToPosition(whichTarget, -1 , new_pos, {0,0,0}, vel, accel)
    
    new_pos[1] = new_pos[1] + relPosnew[1]/2.0
    new_pos[2] = new_pos[2] + relPosnew[2]/2.0
    new_pos[3] = 0

    simMoveToPosition(whichTarget, -1 , new_pos, {0,0,0},vel, accel)

end


-- this converts a vector in absolute coordinates to a vector in relatie 
--coordinates (relative to the vector formed by right_vertex - left_vertex)
-- these are the positions of the joint thats designated right and joint thats designated left
function convertVectorFromActual(x,y, right_v, left_v)
    
    right_vertex = simGetObjectPosition(jointTips[right_v], -1)
    left_vertex = simGetObjectPosition(jointTips[left_v], -1)
    axis_vector = subtractVectors(right_vertex, left_vertex)
    unit_v = getUnitVector(axis_vector)
    perp_v = getPerpVector(unit_v)

    temp1 = {perp_v[2]*x - perp_v[1]*y, unit_v[1]*y - unit_v[2]*x, 0}

    c = 1.0/(unit_v[1]*perp_v[2]-unit_v[2]*perp_v[1])
    

    return multiplyVector(temp1,c)

end


-- this function returns the center of the feet positions. Its used to reposition the
-- hexapod's base to the center of the legs
function getLegCenter()
    average_leg_position = {0,0,0}
    
    for i = 1,6,1 do
        p = simGetObjectPosition(legTargets[i], -1)
        average_leg_position[1] = average_leg_position[1] + p[1]
        average_leg_position[2] = average_leg_position[2] + p[2]
        average_leg_position[3] = average_leg_position[3] + p[3]
    end
    
    
    average_leg_position[1] = average_leg_position[1]/6.0
    average_leg_position[2] = average_leg_position[2]/6.0
    average_leg_position[3] = average_leg_position[3]/6.0
    return average_leg_position
end


-- this function moves the base to the center calculated in getLegCenter()
function positionBaseCenter()
    s = subtractVectors(getLegCenter(), simGetObjectPosition(legBase, -1))

    -- 4 is the right vertex default and 1 is the left vertex default of the simulation environment
    s = convertVectorFromActual(s[1],s[2], 1, 4) 
    legCenter = getLegCenter()
    legCenter[3] = initialZ

    s[3] = -simGetObjectPosition(legBase, -1)[3]

    simMoveToPosition(legBase,antBase,multiplyVector(s,-1.0),{0,0,0},vel,accel)
    
end

-- this function returns where the home position of a given leg should be relative to the robot's center
function getLegHomePosition(leg)
    joint_pos = simGetObjectPosition(jointTips[leg], -1)
    center_pos = centerPosition()

    -- unit vector from center to leg home position
    unit_vector_to_leg = getUnitVector(subtractVectors(joint_pos,center_pos))

    center_to_joint_len = getMagVector(subtractVectors(joint_pos,center_pos))

    -- where the leg should be if it were in home position according to the current physical
    -- orientation (position and angle) of the base
    
    leg_home_position = addVectors(multiplyVector(unit_vector_to_leg, center_to_joint_len * 2.5), center_pos)
    leg_home_position[3] = 0.0

    return leg_home_position
end


-- this functiont takes a leg and an action_index and returns the actual
-- displacement the leg needs to move to perform that action
-- the action_index is an index of the array of possible actions
-- the possible actions are moving to the possible states of that leg
function getActionRelLeg(leg, action_index)
    -- returns the leg action from the robot's frame

    -- where the leg should be if it were in home position according to the current physical
    -- orientation (position and angle) of the base
    leg_home_position = getLegHomePosition(leg)

    
    -- now get unit vector going from the left vertex to right vertex, and then the perp vector
    right_vertex = simGetObjectPosition(jointTips[right_v_num], -1)
    left_vertex = simGetObjectPosition(jointTips[left_v_num], -1)
    axis_vector = subtractVectors(right_vertex, left_vertex)
    unit_v = getUnitVector(axis_vector)
    perp_v = getPerpVector(unit_v)


    -- now generate the new action
    -- actions are defined as x,y movements away from the default home position of a foot
    -- x is in the direction of the vector going from the left vertex to the right vertex
    -- y is in the direction of the perpendicular vector to this vector
    leg_current_p = simGetObjectPosition(legTargets[leg], -1)

    leg_offset = possible_states[action_index]
    addunit= multiplyVector(unit_v, leg_offset[1])
    addperp = multiplyVector(perp_v,leg_offset[2]) 
    new_p_of_leg = addVectors(addVectors(leg_home_position,addunit),addperp)
    action_vector_ffloor = subtractVectors(new_p_of_leg,leg_current_p)
    action_vector_frobot = convertVectorFromActual(action_vector_ffloor[1],action_vector_ffloor[2], right_v_num, left_v_num)

    return {action_vector_frobot[1],action_vector_frobot[2],0}
end

-- Finds the available states by looking at the square surrounding the legs home position
-- an example state would be .1 forward and .1 to the right of the leg home position
-- only needs to be called once
function FindPossibleStates()
    
    joint_pos = simGetObjectPosition(jointTips[1], -1)
    center_pos = centerPosition()

    center_to_joint_len = getMagVector(subtractVectors(joint_pos,center_pos))

    division_length = DIVISIONLENGTHPARAMETER*center_to_joint_len/(SIDE_DIVISIONS - 1.0)
    --division_length = 1.0*center_to_joint_len/(side_divisions - 1.0)

    loop_len = math.floor((SIDE_DIVISIONS - .99)/2.0)


    -- now generate the possible states
    for x = -loop_len,loop_len,1 do
        for y = -loop_len,loop_len,1 do
            table.insert(possible_states, {x * division_length,y * division_length,0})
        end
    end

end


-- Finds the available states by looking at the square surrounding the legs home position
-- an example state would be .1 forward and .1 to the right of the leg home position
-- this version is called when the number of side divisions for the states is an even number
-- For example, if we want to have a state space consisting of the four vertices of a square
-- centered at the leg's home position as well as the leg's home position, then we would 
-- use this function
function FindPossibleStatesEven()
    joint_pos = simGetObjectPosition(jointTips[1], -1)
    center_pos = centerPosition()

    center_to_joint_len = getMagVector(subtractVectors(joint_pos,center_pos))

    division_length = DIVISIONLENGTHPARAMETER*center_to_joint_len/(SIDE_DIVISIONS)

    loop_len = math.floor((SIDE_DIVISIONS + .01)/2.0)

    -- now generate the possible states
    for x = -loop_len,loop_len,1 do
        if (x~=0) then
            for y = -loop_len,loop_len,1 do
                if (y~=0) then
                    table.insert(possible_states, {x * division_length,y * division_length,0})
                end 
            end   
        end
    end
    table.insert(possible_states, {0,0,0})
end

-- get the center position of the hexapod
function centerPosition()
    j1 = simGetObjectPosition(jointTips[1], -1)
    j4 = simGetObjectPosition(jointTips[4], -1)
    return addVectors(multiplyVector(subtractVectors(j4,j1),0.5), j1)
end

-- choose a random element of an array
function randomChoice(array)
    array_length = table.getn(array)
    choose_index = 1 + math.floor(array_length*math.random())
    return array[choose_index]
end


-- randomly choose two symmetrically opposing legs to move
function randomWalk()
    
    for i = 1,10,1 do
        leg = 1 + math.floor(6*math.random())
        actions = getLegalActions(getState())
        
        choosespot = 1 + math.floor(25*math.random())
        action = actions[choosespot]

        -- move foot
        leg = action[1]
        action_index = action[2]
        action_vector = getActionRelLeg(leg, action_index)
        
        --use both if opposite legs should move symmetrically
        moveLegRel(legTargets[leg], stepHeight , action_vector[1], action_vector[2])
        moveLegRel(legTargets[legPairs[leg]], stepHeight , -action_vector[1], -action_vector[2])

        positionBaseCenter()
    end
end

-- flip coin function
function flipCoin(p)
    r = math.random()
    return r < p
end

-- go back to the home position with legs separated by 60 degrees
function goToHomeOrientation()
    for leg = 1,6,1 do
        leg_home_pos = getLegHomePosition(leg)
        leg_pos = simGetObjectPosition(legTargets[leg], -1)
        rel_pos_ffloor = subtractVectors(leg_home_pos,leg_pos)
        rel_pos_frobot = convertVectorFromActual(rel_pos_ffloor[1],rel_pos_ffloor[2], right_v_num, left_v_num)
        moveLegRel(legTargets[leg], stepHeight ,rel_pos_frobot[1], rel_pos_frobot[2])
    end
end

------------------QLEARNING FUNCTIONS----------------------------


-- returns the state which is a list of 6 leg positions relative to the home position
-- of the leg. These positions are is in the robot's frame (with the x-axis being the line
-- connecting the left vertex to the right vertex. 
function getState()
    states = {}
    for leg = 1,6,1 do
        -- get the relative position of the leg
        leg_home_pos = getLegHomePosition(leg)
        leg_pos = simGetObjectPosition(legTargets[leg], -1)
        rel_pos_ffloor = subtractVectors(leg_pos, leg_home_pos)
        rel_pos_frobot = convertVectorFromActual(rel_pos_ffloor[1],rel_pos_ffloor[2], right_v_num, left_v_num)
            
        -- find the relative position that is closest to the current relative position of the leg
        min_distance = 10000000.0
        save_closest_state = 1
        for z = 1,table.getn(possible_states) do
            rel_pos_try = possible_states[z]
            distance = getMagVector(subtractVectors(rel_pos_try,rel_pos_frobot))
            if distance < min_distance then
                min_distance = distance
                save_closest_state = z
            end
        end
        
        -- insert the relative position that gave the closest distance
        table.insert(states, save_closest_state)
    end
    return states

end


-- returns an array of actions, where each action is a tuple of leg number,action_index
-- each leg always has the same options for its legal actions
-- these come from the possible_states array
function getLegalActions(state)
    actions = {}
    for i = 1,NUMBEROFFREELEGS,1 do
        leg = FREELEGS[i]

        for action_index = 1, table.getn(possible_states), 1 do
            if action_index ~= state[leg] then
                table.insert(actions,{leg,action_index})
            end
        end
    end
    return actions
end

-- get the value from the Q-values
function computeValueFromQValues(state)

    actions = getLegalActions(state)
    current_max = -10000000.0
    for i = 1,table.getn(actions),1 do           
        action = actions[i]

        if getQValue(state,action) == nil then
            QValues[{state,action}] = 0.0
        end

        if getQValue(state,action) >= current_max then
            current_max = getQValue(state,action)
        end
    end
    return current_max
end

-- create the string that the dictionary QValues uses to store state,action pairs.
-- Lua dictionaries didnt accept tuples so I made the keys strings
function concatStateAction(state,action)
    state_string = state[1]..","..state[2]..","..state[3]..","..state[4]..","..state[5]..","..state[6]
    action_string = "("..action[1]..","..action[2]..")"
    
    full_string = state_string..action_string
    return full_string
end

-- get the Qvalue from the dictionary. Similar implementation to the Counters we used in class
function getQValue(state,action)

    if QValues[concatStateAction(state,action)] == nil then 
        QValues[concatStateAction(state,action)] = 0.0        
    end
        
    return QValues[concatStateAction(state,action)]
    
end


-- choose the action from the QValues using an epsilon-greedy approach
function computeActionFromQValues(state)

    -- get best action by finding one connected to max qvalue
    actions = getLegalActions(state)
    best_action = actions[1]
    current_max = -10000000.0
    for i = 1,table.getn(actions),1 do
        action = actions[i]

        if getQValue(state,action) >= current_max then

            -- check for tie and then break it with a coin flip
            if getQValue(state,action) == current_max then
                if flipCoin(.5) then
                    best_action = action
                end
            else
                current_max = getQValue(state,action)
                best_action = action
            end
        end
    end

    return best_action
end

-- returns a tuple of the leg and the action it takes
function getAction()
    
    if flipCoin(EPSILON) then
        actions = getLegalActions(getState())
        return randomChoice(actions)
    else
        return computeActionFromQValues(state)
    end
end

-- calculates reward and moves the robot according to the action
function getReward(state, action)
    
    previousCenter = centerPosition()
    
    -- move foot
    leg = action[1]
    action_index = action[2]
    action_vector = getActionRelLeg(leg, action_index)
    
    -- move both the chosen leg, and its opposite leg (to reduce state space size)
    moveLegRel(legTargets[leg], stepHeight, action_vector[1], action_vector[2])
    moveLegRel(legTargets[legPairs[leg]], stepHeight , -action_vector[1], action_vector[2])

    -- move the base center to align with the center of the feet locations
    positionBaseCenter()

    currentCenter = centerPosition()
    
    displacementVector = subtractVectors(currentCenter,previousCenter)
    relativeDisplacementVector = convertVectorFromActual(displacementVector[1],displacementVector[2], right_v_num, left_v_num)
    

    -- the y-value minus the x-value is considered the reward.
    -- we start each episode by orienting the robot forward movement in the positive y-axis
    return relativeDisplacementVector[2] - relativeDisplacementVector[1]
end

-- the update function for QLearning
function update(state, action, nextState, reward)
    sample = reward + DISCOUNT * computeValueFromQValues(nextState)

    QValues[concatStateAction(state,action)] = (1 - ALPHA) * getQValue(state, action) + ALPHA * sample
end


-- observe the episodes for Qlearning. An episode is just a sequence of movements before
-- going back to the home position
function observeEpisodes()
    for episodes = 1, NUMEPISODES, 1 do
        -- decay epsilon as the number of episodes goes on
        decayEpsilon(episodes, NUMEPISODES)
        
        print(EPSILON, "EPSILON")
        for steps = 1,STEPSPEREPISODE,1 do
            state = getState()
            
            action = getAction()
            
            -- inside the reward function is what actually moves the robot in the simulation
            reward = getReward(state, action)

            -- now the current state is the nextState
            nextState = getState()
            
            update(state,action,nextState,reward)
        end
        goToHomeOrientation()
    end
end

-- using a linear decay of epsilon
function decayEpsilon(numEpisodes, total)
    EPSILON = 1 - (numEpisodes/total)
end

-- now that we have the QValues, we can move solely due to them
function moveAccordingToQValues()
    EPSILON = 0.0
    for i = 1,NUMRACESTEPS,1 do
        state = getState()
        action = getAction()

        -- move foot
        leg = action[1]
        action_index = action[2]
        action_vector = getActionRelLeg(leg, action_index)
    
        -- move both the chosen leg, and its opposite leg (to reduce state space size)
        moveLegRel(legTargets[leg], stepHeight, action_vector[1], action_vector[2])
        moveLegRel(legTargets[legPairs[leg]], stepHeight , -action_vector[1], action_vector[2])
        
        positionBaseCenter()
    end
end

--CONSTANTS FOR SIMULATION--
antBase=simGetObjectHandle('hexa_base')
legBase=simGetObjectHandle('hexa_legBase')
sizeFactor=simGetObjectSizeFactor(antBase)
vel= 2
accel= vel*1.5
initialZ=simGetObjectPosition(legBase,-1)[3]
initialO = simGetObjectOrientation(legBase,-1)
initialO[1] = 0.0
initialO[2] = 0.0



-- define constants for walking parameters
stepHeight=0.03*sizeFactor
maxWalkingStepSize=0.11*sizeFactor
walkingVel=0.9
stepSize = 1.0


-- define variables
legTips={-1,-1,-1,-1,-1,-1}
legTargets={-1,-1,-1,-1,-1,-1}
jointTips = {-1,-1,-1,-1,-1,-1}

for i = 1,6,1 do
legTips[i]=simGetObjectHandle('hexa_footTip'..i-1)
legTargets[i]=simGetObjectHandle('hexa_footTarget'..i-1)
jointTips[i] = simGetObjectHandle('hexa_joint1_'..i-1)
hexapod_handle = simGetObjectHandle('hexapod')
end


---VARIABLES FOR QLEARNING----
QValues = {}
possible_states = {}
ALPHA = .3
DISCOUNT = 0.9
EPSILON = 1
NUMEPISODES = 10
STEPSPEREPISODE = 25
NUMRACESTEPS = 50


-- CHANGE CONSTANTS HERE: CONSTANTS THAT TELL LEGS HOW TO MOVE RELATIVE TO ONE ANOTHER ------

-- legs move in pairs, so the legs are paired to the opposite leg which is the index 
-- (LUA indexes by 1 first)
legPairs = {4,6,5,1,3,2}
FREELEGS = {1,6,5}
NUMBEROFFREELEGS = 3
-- define the number of the opposite leg that moves with this leg
-- joints we are using as robot reference frame. Right vertex and left vertex
right_v_num = 6
left_v_num = 2


-- length that the legs are allowed to move away from their home position
DIVISIONLENGTHPARAMETER = .5

-- number of divisions per side of square (there will be side_divisions^2 states per leg)
-- should be odd for convenience, because we want moving only along one axis to be allowed
SIDE_DIVISIONS = 2.0

--fill up possible_states array
--FindPossibleStates()
FindPossibleStatesEven()

-- randomize the random function
math.randomseed(os.time())



---------- READ QDATA If there is any -------

-- Opens a file in read
file = io.open("/Users/williambryk/Desktop/Hexapod-Qdata.txt", "r")

-- sets the default input file as test.lua
io.input(file)

while true do
  local line = io.read()
  if (line == nil) then break end
  s_key = line
  f_value = tonumber(io.read())
  QValues[s_key] = f_value
end

io.close(file)


---------- Looking Ahead Algorithms-----------


--- GREEDY SEARCH ----


-- gets the closest state given the relative positions of the feet to their home positions
function getClosestState_Simulated(relPositionsArray)
    states = {}
    for leg = 1,6,1 do
        -- get the relative position of the leg
        rel_pos_frobot = relPositionsArray[leg]

        -- find the relative position that is closest to the current relative position of the leg
        min_distance = 10000000.0
        save_closest_state = 1
        for z = 1,table.getn(possible_states) do
            rel_pos_try = possible_states[z]
            distance = getMagVector(subtractVectors(rel_pos_try,rel_pos_frobot))
            if distance < min_distance then
                min_distance = distance
                save_closest_state = z
            end
        end
        
        -- insert the relative position that gave the closest distance
        table.insert(states, save_closest_state)
    end
    return states

end


-- return a list of tuples of the next state and the reward for moving there
function getNextStatesAndRewards_Simulated(state)
    nextStatesAndReward_array = {}
    actions = getLegalActions(state)

    for i = 1, table.getn(actions), 1 do
        --print(concatStateAction(state,actions[i]))
        legThatMoves = actions[i][1]
        action_movement = getActionRelLeg(legThatMoves, actions[i][2])

        -- two legs move foward with y-component actions[i][2][2] so the center average moves 2/6 of this 
        -- because there are six legs
    
        centerDisplacent = action_movement[2]*2.0/6.0

        -- get new relative positions of the new state after moving the two legs according to the action
        -- (two legs move because it is symmetric)
        relPositionsArray = {}
        for leg = 1,6,1 do
            if leg == legThatMoves then
                table.insert(relPositionsArray,addVectors(addVectors(possible_states[state[leg]],action_movement),{0,-centerDisplacent,0}))
            elseif leg == (legPairs[leg]) then
                symmetricAction = action_movement
                symmetricAction[1] = -1 * symmetricAction[1]
                table.insert(relPositionsArray,addVectors(addVectors(possible_states[state[leg]],symmetricAction),{0,-centerDisplacent,0}))
            else
                table.insert(relPositionsArray,addVectors(possible_states[state[leg]],{0,-centerDisplacent,0}))
            end
        end

        nextState = getClosestState_Simulated(relPositionsArray)
        
        reward = centerDisplacent

        table.insert(nextStatesAndReward_array,{nextState,reward})
    end
    return nextStatesAndReward_array
end


-- move greedily
function greedyMove(state)
    rewardsFromActions = getNextStatesAndRewards_Simulated(state)
    max_reward = -10000000.0
    save_best_action = 1
    for z = 1,table.getn(rewardsFromActions) do
        reward_get = rewardsFromActions[z][2]
        if reward_get > max_reward then
            max_reward = reward_get
            save_best_action = z
        end
    end
    action = getLegalActions(state)[save_best_action]
    leg = action[1]
    action_index = action[2]
    action_vector = getActionRelLeg(leg, action_index)

    -- move both the chosen leg, and its opposite leg (to reduce state space size)
    moveLegRel(legTargets[leg], stepHeight, action_vector[1], action_vector[2])
    moveLegRel(legTargets[legPairs[leg]], stepHeight , -action_vector[1], action_vector[2])
    positionBaseCenter()
end


--- UNIFORM COST SEARCH -----

-- 2 moves ahead
function UCSMove2generations(state)
    rewardsFromActions = getNextStatesAndRewards_Simulated(state)
    max_reward = -10000000.0
    save_best_action = 1
    for z = 1,table.getn(rewardsFromActions) do
        reward_get = rewardsFromActions[z][2]
        
        rewardsFromActions2 = getNextStatesAndRewards_Simulated(rewardsFromActions[z][1])
        max_reward2 = -10000000.0
        for z2 = 1,table.getn(rewardsFromActions2) do
            reward_get2 = rewardsFromActions2[z2][2]
            if reward_get2 > max_reward2 then
                max_reward2 = reward_get2
            end
        end

        if reward_get + max_reward2 > max_reward then
            max_reward = reward_get + max_reward2
            save_best_action = z
        end
    end

    action = getLegalActions(state)[save_best_action]
    leg = action[1]
    action_index = action[2]
    action_vector = getActionRelLeg(leg, action_index)

    -- move both the chosen leg, and its opposite leg (to reduce state space size)
    moveLegRel(legTargets[leg], stepHeight, action_vector[1], action_vector[2])
    moveLegRel(legTargets[legPairs[leg]], stepHeight , -action_vector[1], action_vector[2])
    positionBaseCenter()
end



-- recursive version of these functions. It takes too long to run with this simulation
-- I don't actually use these, but they are here to show that I could have!

-- function returns the reward of going down that path, given the number of steps
-- we are allowed to look ahead (howManyBranchesLeft)
function getSumRewards_Simulated_rec(state, howManyBranchesLeft)

    rewardsFromActions = getNextStatesAndRewards_Simulated(state)

    if howManyBranchesLeft == 0 then
        return 0
    else
        local max_sum_reward = -10000000.0    
        for i = 1, table.getn(rewardsFromActions) do
            nextState = rewardsFromActions[i][1]
            nextReward = rewardsFromActions[i][2]
            futureReward = getSumRewards_Simulated_rec(nextState, howManyBranchesLeft - 1)
            
            if nextReward + futureReward > max_sum_reward then
                max_sum_reward = nextReward + futureReward
            end
      
        end
        return max_sum_reward
    end
end
function uniformCostMove(state, howManyBranchesLeft)
    rewardsFromActions = getNextStatesAndRewards_Simulated(state)

    max_reward = -10000000.0
    save_best_action = 1
    for z = 1,table.getn(rewardsFromActions) do
        reward_get = rewardsFromActions[z][2] + getSumRewards_Simulated_rec(rewardsFromActions[z][1], howManyBranchesLeft)
        if reward_get > max_reward then
            max_reward = reward_get
            save_best_action = z
        end
    end
    action = getLegalActions(state)[save_best_action]
    leg = action[1]
    action_index = action[2]
    action_vector = getActionRelLeg(leg, action_index)

    -- move both the chosen leg, and its opposite leg (to reduce state space size)
    moveLegRel(legTargets[leg], stepHeight, action_vector[1], action_vector[2])
    moveLegRel(legTargets[legPairs[leg]], stepHeight , -action_vector[1], action_vector[2])
    positionBaseCenter()
end



-----------------------RACING----------------------------



-- Greedy
--[[
for i = 1,NUMRACESTEPS, 1 do
    greedyMove(getState())
end
--]]



-- UCS 2 generations ahead
--[[
for i = 1,NUMEPISODES, 1 do
    UCSMove2generations(getState())
end
--]]


-- Observe episodes
observeEpisodes()
goToHomeOrientation()
simWait(8)
print("DICTIONARY")
for key,value in pairs(QValues) do
   print(key)
   print(value)
end


startPosition = centerPosition()
-- move according to Q-values
moveAccordingToQValues()

endPosition = centerPosition()

print(subtractVectors(endPosition,startPosition)[2])










