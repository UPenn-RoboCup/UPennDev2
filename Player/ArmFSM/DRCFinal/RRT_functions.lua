-- local start_node = {0, 0, 0}
-- local end_node = {90, 70, 50}
-- local test_end = 1
-- tree_f, tree_b = RRT_bidirectional(start_node, end_node, test_end)
----------------- RRT Functions -----------------------------
--function RRT_bidirectional(qLArmTarget, start_node, end_node, test_end)

function run_RRT_code()

tree_f, tree_b, parent_idx = RRT_bidirectional({0, 0, 0}, {0, 90, 90}, 1)

local path_pointA = {}
local path_pointB = {}
local path_points = {}
local temp_path_points = {}
local test_path_points = {}
path_pointA = find_path3D(tree_f)
path_pointA = vector_flip(path_pointA)
path_pointB = find_path3D(vector_extract_part(tree_b,1,parent_idx))
path_points = vector_fusion_vectors(path_pointA, path_pointB)

  for idx = 1, table.getn(path_points) do
    print(path_points[idx][1], path_points[idx][2], path_points[idx][3])
  end
rrt_idx = 1;

temp_path_points[1]={0, 0, 0, 0, 0, 0, 0}
temp_path_points[2]={-0.00644923716896341,0.0185762812075625,-0.00493399351780154,0.00718622813529728,-0.0132821932246919,0.0250982763260679,0.00801247963310392}
temp_path_points[3]={-0.00983285101215483,0.0315392565629678,0.00346083644796891,0.00937076577996662,-0.0263794186484491,0.0364153173602538,0.0179045543264143}
temp_path_points[4]={-0.182974869951481,0.694872615236823,0.433058837746844,0.121151105991779,-0.696589966104260,0.615511927451300,0.524104019223501}
temp_path_points[5]={-0.205839384090003,0.782470980334795,0.489793182461918,0.135911968467193,-0.785097348351697,0.691984828148424,0.590952553475953}
temp_path_points[6]={-1.16075319000514,4.44093084371416,2.85924973778395,0.752384392558603,-4.48152215259855,3.88579939557907,3.38281688561145}
temp_path_points[7]={-2.11566699592028,8.09939070709352,5.22870629310598,1.36885681665001,-8.17794695684540,7.07961396300972,6.17468121774695}
temp_path_points[8]={-3.52918484228826,13.5148516662615,8.73611139700760,2.28139437547912,-13.6496059045386,11.8072802630281,10.3073581747021}
temp_path_points[9]={-3.63763385056349,13.9303408458881,9.00520924116641,2.35140678708270,-14.0694067496587,12.1699999294339,10.6244286090481}
temp_path_points[10]={-4.30023426647436,16.4688915454611,10.6493400173481,2.77916774660171,-16.6343008311972,14.3861400697843,12.5616616878536}
temp_path_points[11]={-4.31282526902596,16.5171301646173,10.6805826143136,2.78729621270719,-16.6830400791149, 14.4282520409264,12.5984738594695}
temp_path_points[12]={-5.19605308202982, 19.9012983080493,12.8733362588902,3.35732673966301,-20.1026336137024, 17.3821163400518,15.1813622024776}
temp_path_points[13]={-5.19760029967067, 19.9072267556948,12.8771777878247,3.35832530370583,-20.1086241977978, 17.3872908581267,15.1858870296357}
temp_path_points[14]={-5.26412017771133, 20.1624137497005,13.0428184384203,3.40133044676856,-20.3665957125876, 17.6098809839971,15.3807429670684}
temp_path_points[15]={-5.32055550186022, 20.4766098518203,13.2861604536018,3.48420846047037,-20.7036760534948, 17.8651262354532,15.6277618932629}
temp_path_points[16]={-5.45729808012199, 21.2379058261344,13.8757772416180,3.68502157611581,-21.5204204523648, 18.4835845202167,16.2262878228415}
temp_path_points[17]={-5.64752241132776, 22.4029776824843,14.8075769996146,4.01472218436337,-22.7848984101134, 19.4159897410284,17.1475726143356}
temp_path_points[18]={-5.64859124785524, 22.4097288899182,14.8130282232638,4.01667200229276,-22.7922511987573, 19.4213680063766,17.1529204883098}
temp_path_points[19]={-5.65505754969608, 22.4505726896344,14.8460073146512,4.02846811043654,-22.8367344752182, 19.4539057109062,17.1852743295522}
temp_path_points[20]={-5.72032440227676, 26.3329316580173,18.8276888920065,5.79981419169302,-27.4826014261053, 22.1424708949086,20.4118058884650}
temp_path_points[21]={-5.71633706173646, 26.3570597578281,18.8592737552727,5.81661119773056,-27.5145908193422, 22.1559176264937,20.4329213750111}
temp_path_points[22]={-5.71412658120619, 26.3683950447027,18.8749156688647,5.82526968166195,-27.5297689320044, 22.1618409618594,20.4429109938687}
temp_path_points[23]={-5.71409434279190, 26.3686423790800,18.8753184581936,5.82546284818524,-27.5301224356160, 22.1619982953368,20.4430780938026}
temp_path_points[24]={-5.71380541978195, 26.3708504447115,18.8789172833476,5.82719143954612,-27.5332828886220, 22.1634063619103,20.4445731479929}
temp_path_points[25]={-5.71302832256943, 26.3767893283861,18.8885967998649,5.83184071640179,-27.5417833464421, 22.1671935446935,20.4485942953757}
temp_path_points[26]={-5.71295207043146, 26.3773720773266,18.8895465958711,5.83229692354615,-27.5426174481118, 22.1675651594405,20.4489888677407}
temp_path_points[27]={-5.71213917038610, 26.3835845802547,18.8996720720132,5.83716040429824,-27.5515095433928, 22.1715268272760,20.4531952795100}
temp_path_points[28]={-5.68662514374136, 26.5785728399901,19.2174745736652,5.98980768016678,-27.8306006286647, 22.2958694193021,20.5852195087559}
temp_path_points[29]={-5.54192493450252, 27.6844289392912,21.0198591649320,6.85553119239275,-29.4134373560013, 23.0010657935949,21.3339815078903}
temp_path_points[30]={-5.51738265413594, 27.8719907305071,21.3255576031028,7.00236463008272,-29.6818987697143, 23.1206725807754,21.4609773637171}
temp_path_points[31]={-5.37178123478882, 28.9847342330832,23.1391676599111,7.87347997127907,-31.2745935923911, 23.8302610019365,22.2144027416114}
temp_path_points[32]={-5.30938100879485, 29.4616213839463,23.9164243509327,8.24681283026637,-31.9571729059727, 24.1343684621639,22.5372973539099}
temp_path_points[33]={-5.05471912268616, 31.4078480099770,27.0884906659695,9.77042349836306,-34.7428508846432, 25.3754629961600,23.8550642317128}
temp_path_points[34]={-4.80005723657746, 33.3540746360077,30.2605569810062,11.2940341664598,-37.5285288633137, 26.6165575301562,25.1728311095157}
temp_path_points[35]={-4.54539535046876, 35.3003012620384,33.4326232960430,12.8176448345564,-40.3142068419843, 27.8576520641523,26.4905979873186}
temp_path_points[36]={-4.29073346436006, 37.2465278880691,36.6046896110798,14.3412555026531,-43.0998848206548, 29.0987465981484,27.8083648651215}
temp_path_points[37]={-4.03607157825137, 39.1927545140998,39.7767559261165,15.8648661707498,-45.8855627993253, 30.3398411321445,29.1261317429244}
temp_path_points[38]={-3.78140969214267, 41.1389811401306,42.9488222411533,17.3884768388465,-48.6712407779959, 31.5809356661407,30.4438986207273}
temp_path_points[39]={-3.52674780603397, 43.0852077661613,46.1208885561901,18.9120875069432,-51.4569187566664, 32.8220302001368,31.7616654985302}
temp_path_points[40]={-3.34506809301495, 44.4736758258705,48.3838895360352,19.9990548311951,-53.4442643444244, 33.7074461617136,32.7017806914375}
temp_path_points[41]={-3.04604020078203, 46.7589649780177,52.1085784508248,21.7881018441827,-56.7152501488961, 35.1647583972959,34.2491227689856}
temp_path_points[42]={-2.74701230854911, 49.0442541301650,55.8332673656145,23.5771488571703,-59.9862359533677, 36.6220706328782,35.7964648465338}
temp_path_points[43]={-2.44798441631619, 51.3295432823122,59.5579562804042,25.3661958701579,-63.2572217578393, 38.0793828684604,37.3438069240819}
temp_path_points[44]={-2.14895652408326, 53.6148324344594,63.2826451951939,27.1552428831455,-66.5282075623110, 39.5366951040427,38.8911490016301}
temp_path_points[45]={-1.84992863185034, 55.9001215866066,67.0073341099836,28.9442898961331,-69.7991933667826, 40.9940073396250,40.4384910791782}
temp_path_points[46]={-1.55090073961742, 58.1854107387539,70.7320230247733,30.7333369091207,-73.0701791712542, 42.4513195752073,41.9858331567264}
temp_path_points[47]={-1.25187284738449, 60.4706998909011,74.4567119395630,32.5223839221083,-76.3411649757259, 43.9086318107895,43.5331752342745}
temp_path_points[48]={-1.23479083105570, 60.6012474004377,74.6694853915209,32.6245835198134,-76.5280205647012, 43.9918810061020,43.6215673987564}
temp_path_points[49]={-0.955866868321615,62.7328943901226,78.1437599292809,34.2933512011620,-79.5790948836129, 45.3512167545306,45.0648801972786}
temp_path_points[50]={-0.933703452740756,62.9022759579197,78.4198272476553,34.4259521841220,-79.8215345343165, 45.4592301457467,45.1795664402721}
temp_path_points[51]={-0.788369675066697,64.0129740372159,80.2301035620315,35.2954662581851,-81.4113016966112, 46.1675142153907,45.9316068864153}
temp_path_points[52]={-0.538246031546429,65.9245176225453,83.3456415654933,36.7919251803814,-84.1473370584236, 47.3864916271252,47.2258902902762}
temp_path_points[53]={-0.528388970919170,65.9998491695636,83.4684210298991,36.8508987588672,-84.2551607973559, 47.4345300055913,47.2768963839071}
temp_path_points[54]={-0.299968976883881,67.7455249010294,86.3136185576162,38.2175074219720,-86.7537857663101, 48.5477346958747,48.4588726386466}
temp_path_points[55]={-0.0715489828485916, 69.4912006324952,89.1588160853333,39.5841160850768,-89.2524107352643,49.6609393861581,49.6408488933860}
temp_path_points[56]={-0.0550260524295884, 69.6174753868878,89.3646255678934,39.6829707407117,-89.4331506336238, 49.7414638766580,49.7263480262288}
temp_path_points[57]={-0.0252656954721882, 69.8449157785212,89.7353203242643,39.8610232873237,-89.7586911857668, 49.8865009566623,49.8803452076260}
temp_path_points[58]={-0.0159569396785507, 69.9160569636334,89.8512701082660,39.9167164255886,-89.8605171653795, 49.9318671718831,49.9285140570986}
temp_path_points[59]={-0.00523026673150723,69.9980344310395,89.9848814562707,39.9808927873347,-89.9778533598217, 49.9841436053872,49.9840200243472}
temp_path_points[60]={-0.00516965558132276,69.9984976410773,89.9856364226611,39.9812554150346,-89.9785163650079, 49.9844389927451,49.9843336600003}
temp_path_points[61]={-0.00437376216128654,70.0045776616877,89.9955467606614,39.9860163431805,-89.9872201140006, 49.9883171435383,49.9884513373411}
temp_path_points[62]={0, 70,90,40,-90, 50,50}


test_path_points[1] ={0, 0, 0, 0, 0, 0, 0}




--return rrt_idx, path_points
return rrt_idx, temp_path_points
--return rrt_idx, test_path_points
end

function vector_fusion_vectors(vector1, vector2)
local size_vector1 = table.getn(vector1)
local size_vector2 = table.getn(vector2)
local output_vector = {}
output_vector = vector1

for idx = 1, size_vector2 do
--  print(idx)
  output_vector[size_vector1 + idx] = vector2[idx]    
end

return output_vector
end


function vector_flip( input_vector )
local size_input_vector = table.getn(input_vector)
local output_vector = {}

for idx =1, size_input_vector do
  output_vector[idx] = input_vector[size_input_vector - idx + 1]
end

return output_vector;

end



function find_path3D(tree)

local find_root = 1;
local node_idx = table.getn(tree);
local path_point = {};    --local path_point = [0 0];
--path_point[1] = {0,0};
local size_path_point = 1;

    while (find_root==1) do
--      print(node_idx)
        path_point[size_path_point] = vector_extract_part(tree[node_idx],1,3);
        node_idx = tree[node_idx][5];
        size_path_point = table.getn(path_point)
        size_path_point = size_path_point + 1;
        if node_idx == 0 then
            find_root = 0;
        end
    end

return path_point

end


-- function generate_tree( tree_f, tree_b, parent_idx )
--   total_tree = tree_f;
--   size_tree_f = table.getn(tree_f);
--   size_tree_b = table.getn(tree_b);
  
--   total_tree = tree_f 
--   for idx = 1, parent_idx do
--     total_tree[size_tree_f + idx] = tree_b[parent_idx - idx + 1];
--   end

--   return total_tree

-- end

function RRT_bidirectional(start_node, end_node, test_end)
 
--  local qLArmTarget = {};
-- qLArmTarget[1] = 0.5;
-- qLArmTarget[2] = 0;
-- qLArmTarget[3] = 0;
-- qLArmTarget[4] = 0;
-- qLArmTarget[5] = 1.07;
-- qLArmTarget[6] = 0;
local tree_f={};
local tree_b={};
local P = {L =5, SZR =5, INC =4}
-- local Param_RRT = {};
 
-- Param_RRT[1] = 5; --RRT tree extension length  
-- Param_RRT[2] = 5; --Safety Zone of Radius     
-- Param_RRT[3] = 4;     --Increment of d

-- establish tree starting from the start node
-- local start_node = {};
-- local end_node = {};
-- start_node[1] = 0; --[theta1, theta2, theta3]
-- start_node[2] = 0; --[theta1, theta2, theta3]
-- start_node[3] = 0; --[theta1, theta2, theta3]


-- --end_node   = [90 70 50]; -- [x y theta]
-- end_node[1] = 90; --[theta1, theta2, theta3]
-- end_node[2] = 70; --[theta1, theta2, theta3]
-- end_node[3] = 50; --[theta1, theta2, theta3]

--first node of forward tree  tree = [start_node 0 0];   -- states, cost, parent_idx
-- tree_f[1][1] = start_node[1];
-- tree_f[2][1] = start_node[2];
-- tree_f[3][1] = start_node[3];
-- tree_f[4][1] = 0;
-- tree_f[5][1] = 0;

--first node of backward tree  tree = [end_node 0 0];   -- states, cost, parent_idx
-- tree_b[1][1] = end_node[1];
-- tree_b[2][1] = end_node[2];
-- tree_b[3][1] = end_node[3];
-- tree_b[4][1] = 0;
-- tree_b[5][1] = 0;

tree_f[1] = {start_node[1], start_node[1], start_node[3], 0, 0}
tree_b[1] = {end_node[1], end_node[2], end_node[3], 0, 0};

sigma_f = generate_samples(start_node, end_node);
sigma_b = generate_samples(end_node, start_node);

   -- if test_end == 1 then
 
   --   local tree = {};
  
   --   tree[1] = { 90,  70,  50,  0,   0}
   --   tree[2] = {86.8477915248646,    68.3984991664523,    46.4646390835687,    5,   1}
   --   tree[3] = {87.5752578212372,    67.2859002229858,    46.5715473217587,    5,   1}
   --   tree[4] = {86.6063600046944,    68.6029476594729,    46.6042007803626,    5,   1}
   --   tree[5] = {82.7217761211528,    66.5896312888730,    45.5923001832204,    10,  3}
   --   tree[6] = {82.9958694514196,    68.4601171159839,    43.2773362161896,    10,  2}
   --   tree[7] = {82.3055265722868,    67.5843011731541,    48.9419756032648,    10,  4}
   --   tree[8] = {89.5502261471316,    69.9388754078450,    50.3792384433148,    10,  2}
   --   tree[9] = {79.4157286299750,    67.8352250050576,    46.7112991671149,    15,  6}
   --   tree[10] = {79.9669057818045,    64.0978187026659,    42.2454270979961,    15,  5}
   --   tree[11] = {77.2775835533374,    64.8067887029908,    43.3561673968897,    20,  9}
   --   tree[12] = {75.7867403115658,    64.9037482361386,    48.5104344422230,    20,  9}
   --   tree[13] = {75.7775497496578,    60.6158641846753,    45.9386712542956,    25,  12}
   --   local randomPoint ={};
   --   randomPoint = {54.4526050611374, 44.6889310470880,    56.6057982949417};
   --   P = {L =5, SZR =5, INC =4}
   --   --find_NearestNeighbor(tree, randomPoint, P)
   --   new_node, parent_idx=find_NearestNeighbor(tree, randomPoint, P);

   --    -- check to see if ostacle or ramdon point is reached
   --    if collision_manipulator(new_node)==0 then
   --       local new_tree = {}
   --       new_tree = tree;
      
   --       new_tree[table.getn(tree)+1] = new_node;
   --      flag1=1;
   --    end
   -- end

local Max_Ite = 3;   -- k-nearest neighbor
local Max_Ite_a = 3;
local Max_Ite_b = 3;

local numPaths_a = 0;  
local numPaths_b = 0;  
local INDC = 0;
local INDC_a = 0;
local INDC_b = 0;
local Ite_num = 0;


while ((numPaths_a < 1) or (numPaths_b<1)) do
    if numPaths_a < 1 then
        tree_f,flag,INDC, num_i = extendTree_bidirectional(tree_f,end_node,world,P,sigma_f,Max_Ite,INDC);  -- Two Phase Sampling
        numPaths_a = numPaths_a + flag;
--        print("tree_f", tree_f)
    end
    
    if numPaths_b < 1 then
      tree_b,flag,INDC, num_i = extendTree_bidirectional(tree_b,start_node,world,P,sigma_b,Max_Ite,INDC);  -- Two Phase Sampling
      numPaths_b = numPaths_b + flag;
    end
    
    local size_tree_f = table.getn(tree_f);    -- can't find the cause of the error
    last_node_f = vector_extract_part(tree_f[size_tree_f],1,3);
    btw_node, parent_idx, child_idx = find_NearestNeighbor(tree_b, last_node_f, P);
    close_node = vector_extract_part(btw_node,1,3)
    length =  vector_distacne(close_node, last_node_f);
    if (length < P["L"]) then
          print("finished!!!!")
           break;       
    end
end

return tree_f, tree_b, parent_idx;

--generate_bidirectional_RRT_Plot(tree, tree_b, end_node, start_node,  parent_idx);


end

-- extendTree : extend tree by randomly selecting point and growing tree toward end point
function extendTree_bidirectional(tree,end_node,world,P,sigma,Min_Ite,INDC)

    --initialize
    local flag1 = 0;
    local ite_num=0;
    local randomPoint = {};
    local new_tree = {};

    while flag1==0 do        --comment due to the error
            
         --select a biased random point
          ite_num = ite_num + 1;  
          randomPoint = generate_RandomPoint(sigma, end_node);       
            
          for xxx = 1, table.getn(randomPoint) do
--            print(randomPoint[xxx])
          end  
         -- find node that is closest to randomPoint
         new_node, parent_idx=find_NearestNeighbor(tree, randomPoint, P);
           
         -- check to see if ostacle or ramdon point is reached
         if collision_manipulator(new_node)==0 then
           
            new_tree = tree;            
            new_tree[table.getn(new_tree)+1] = new_node;
           flag1=1;
         end
    end    -- end  --End while

--    print(P["L"], INDC, end_node[1], new_node[1])
    -- for aa = 1, 2 do
    --     print(new_tree[aa][1]) 
    -- end

    flag =  check_terminate(new_node, end_node, INDC, P);
    return new_tree, flag, INDC, ite_num

end

function collision_manipulator( node )
   
--   print("asdf")
    local collision_flag = 0;

    local radi = 1;
    local xr = 8; 
    local yr = 8;
    local cpt = {xr, yr};
    local length1 = 4;
    local length2 = 4;
    local length3 = 4;

    local pitodeg = 1*3.141592/180;

    local theta1 = node[1] * pitodeg;
    local theta2 = node[2] * pitodeg;
    local theta3 = node[3] * pitodeg;

    local pt1 = {length1*math.cos(theta1), length1*math.sin(theta1)};
    local pt2 = {length2*math.cos(theta2), length2*math.sin(theta2)};
    pt2 = vector_sum(pt1, pt2)
    local pt3 = {length3*math.cos(theta3), length3*math.sin(theta3)};
    pt3 = vector_sum(pt2, pt3)   
    dis_to_line = math.abs(-(pt3[2] - pt2[2])*cpt[1] + (pt3[1] - pt2[1])*cpt[2] + pt2[1]*(pt3[2] - pt2[2]) - (pt3[1] - pt2[1])*pt2[2])/math.sqrt((pt3[2] - pt2[2])^2 + (pt3[1] - pt2[1])^2);

--    print("dis_to_line",dis_to_line,"vector_distacne(pt2,cpt)",vector_distacne(pt2,cpt))
--    print("length2", length2,"dis_to_line",dis_to_line)

    if (vector_distacne(pt2,cpt) > (length2+radi)) then
        collision_flag = 0;

    elseif (dis_to_line > radi) then
        collision_flag = 0;
    else 
        collision_flag = 1;
    end
--    print("collision_flag",collision_flag)
    return collision_flag;
end


function find_NearestNeighbor(tree, randomPoint, P)

-- find node that is 3rd closest to randomPoint 
    local tmp = {}
    local dis_tmp = {}
    local tree_size = table.getn(tree)
    local tree_point = {}

    for idx = 1, tree_size do

         tmp_vector =  vector_sum(vector_extract_part(tree[idx],1,3), vector_times_constant(randomPoint,-1))
        dis_tmp[idx] = (vector_calc_norm(tmp_vector))^2; 
        tmp[idx] = {tree[idx][1] - randomPoint[1], tree[idx][2] - randomPoint[2], tree[idx][3] - randomPoint[3], dis_tmp[idx], idx}
    --    tree_point[idx] = {tree[idx][1], tree[idx][2], tree[idx][3]}
    end

     for xxx = 1, table.getn(tmp) do
            for yyy = 1, table.getn(tmp[xxx]) do
--                            print("tmp",tmp[xxx][yyy])

            end
     end

        table.sort(tmp, compare) 
     for xxx = 1, table.getn(tmp) do
            for yyy = 1, table.getn(tmp[xxx]) do
--                            print("tmp2",tmp[xxx][yyy])

            end
     end

        child_idx = 1;
        kk = tree_size;
        parent_idx = tmp[kk][5];
--        print("child_idx",child_idx, "kk",kk, "parent_idx",parent_idx)

        if tree_size > 3 then
            kk = 3;
            parent_idx = tmp[kk][5];
        end

 --      print("child_idx",child_idx, "kk",kk, "parent_idx",parent_idx)
 
        cost = tree[parent_idx][4] + P["L"];
   --     print("tree[parent_idx][4]",tree[parent_idx][4])
 --       print("parent_idx", parent_idx)
 --       print("cost", cost)
        local extend_length = {}
        extend_length = {randomPoint[1] - tree[parent_idx][1], randomPoint[2] - tree[parent_idx][2], randomPoint[3] - tree[parent_idx][3]}

        for xxx = 1, table.getn(extend_length) do
--            print("extend_length",extend_length[xxx])
        end

 --       print("norm", vector_calc_norm(extend_length))
--        print("P[L]/vector_calc_norm(extend_length)",P["L"]/vector_calc_norm(extend_length))
        local new_point = {};
        new_point = vector_times_constant(extend_length, P["L"]/vector_calc_norm(extend_length))
        new_point = vector_sum(new_point, vector_extract_part(tree[parent_idx],1,3))

        for xxx = 1, table.getn(new_point) do
--            print("new_point",new_point[xxx])
        end

        local new_node = new_point;
        new_node[4] = cost;
        new_node[5] = parent_idx; 

        for xxx = 1, table.getn(new_node) do
 --           print("new_node",new_node[xxx])
        end

    return new_node, parent_idx, child_idx  
end

function vector_extract_part( input_vector, start_idx, end_idx)
    local vector_size = table.getn(input_vector)
    if start_idx < 1 then
        print("Input vector size or start index is wrong")
        return
    end
    if end_idx > vector_size then
        print("The end index exceeds the size of input_vector")
        return
    end

    local output_vector = {}
    for i = start_idx, end_idx do
        output_vector[i - start_idx+1] = input_vector[i];
    end
    -- body
    return output_vector;
end


function vector_calc_norm(input_vector)
    local vector_size = table.getn(input_vector)
    local sum_val = 0
    for i = 1, vector_size do
        sum_val = sum_val + input_vector[i] * input_vector[i];
    end
    return math.sqrt(sum_val)
end

function vector_sum( vector1, vector2 )
    local vector_size1 = table.getn(vector1)
    local vector_size2 = table.getn(vector2)

    if (vector_size1 ~= vector_size2) then
        print("Two vectors have same size!!!")
        return
    end
    local sum_vector = {}

    for i=1, vector_size1 do
        sum_vector[i] = vector1[i]+vector2[i];
    end

    return sum_vector;    -- body
end

function vector_distacne(vector1, vector2)
    local vector_size1 = table.getn(vector1)
    local vector_size2 = table.getn(vector2)

    if (vector_size1 ~= vector_size2) then
        print("Two vectors have same size!!!")
        return
    end
    local diff_vector = {}

    for i=1, vector_size1 do
        diff_vector[i] = vector1[i]-vector2[i];
    end
    length = vector_calc_norm(diff_vector)
    return length;    -- body

end

function vector_times_constant(input_vector, constant)
    local vector_size = table.getn(input_vector)
    local output_vector = {}
    for i=1, vector_size do
        output_vector[i] = input_vector[i] * constant;
    end
    return output_vector;
    -- body
end

function generate_RandomPoint(sigma, end_node)
  local randomPoint = {};
      -- select a biased random point
    p=math.random();   -- generate random value between 0 and 1 

    if p < 0 then   -- p < 0.1 %.05
        local randomPoint = end_node;
    else
           local r     = sigma[1]*(math.random());
           local nrand_v = randn(1);
           local theta = sigma[2] + sigma[5]*(nrand_v[1]);
                  nrand_v = randn(1);
           local theta2 = sigma[3] + sigma[5]*(nrand_v[1]);
           
            -- theta = sigma.theta*(randn-0.5);

            Sx = sigma[6] + r * math.cos(theta);
            Sy = sigma[7] + r * math.sin(theta);  
            Sz = sigma[8] + r * math.sin(theta2);  
            --Sz = sigma.sp(3) + (r-sigma.r/2) ;  
            palt = math.random();
            if p < 0.3 then
                randomPoint[1] = Sx;--[Sx,Sy,end_node(3)];--[Sx,Sy,end_node(3)+10*rand];
                randomPoint[2] = Sy;
                randomPoint[3] = Sz;
            else
                randomPoint[1] = Sx;--[Sx,Sy,end_node(3)];--[Sx,Sy,end_node(3)+10*rand];
                randomPoint[2] = Sy;
                randomPoint[3] = Sz;
            end
    end

    return randomPoint; 
end

function compare(a,b)
  return a[4] < b[4]
end


function generate_samples(start_node, end_node)

    --Diff = start_node(1:3)-end_node(1:3);
    psi = math.atan2(end_node[2]-start_node[2],end_node[1]-start_node[1]); 
    if psi > math.pi then
      psi = psi - 2*math.pi;
    end

    roll = math.atan2(end_node[3]-start_node[3],end_node[1]-start_node[1]); 
    if roll > math.pi then
      roll = roll - 2*math.pi;
    end

    pai = math.atan2(end_node[3]-start_node[3],end_node[2]-start_node[2]); 
    if pai > math.pi then
      pai = pai - 2*math.pi;
    end

    local sigma = {};
    local temp_val;
    temp_val = (end_node[1]-start_node[1])*(end_node[1]-start_node[1]);
    temp_val = temp_val + (end_node[2]-start_node[2])*(end_node[2]-start_node[2]);
    temp_val = temp_val + (end_node[3]-start_node[3])*(end_node[3]-start_node[3]);
    sigma[1] = math.sqrt(temp_val);
    sigma[2] = psi; 
    sigma[3] = roll; 
    sigma[4] = pai;
    sigma[5] = 0.150*math.pi;
    sigma[6] = start_node[1];
    sigma[7] = start_node[2];
    sigma[8] = start_node[3];
      return sigma;
end

function check_terminate(new_node, end_node, INDC, P)
    -- check to see if new node connects directly to end_node
    local Term;
    local flag;
    Term = vector_distacne(vector_extract_part(new_node,1,3), vector_extract_part(end_node,1,3))
--    Term = norm(new_node(1:3)-end_node(1:3));  
    --print("Term",Term)
    if (( Term <= 7*P["L"] ) and INDC ==0 ) then 
          INDC  =  1;
    end

    if INDC == 1 then
        if (Term < 10) then
              flag = 1;
        else
              flag = 0; 
        end
      
    else
        flag = 0;
    end  

    return flag;
end





-- ---Table of normal distributed random numbers.
-- --@param n length of table to return
-- --@return table of n normally distributed random numbers
 function randn(n)
   local t = {}
   for i = 1,n do
     --Inefficient implementation:
     t[i] = math.sqrt(-2.0*math.log(1.0-math.random())) *
                       math.cos(math.pi*math.random())
   end
   return t
 end


--------------------------------------------------------------




