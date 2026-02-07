# ===============================
# 1) Generate Manhattan Road Network
# ===============================
netgenerate \
  --grid \
  --grid.x-number 4 \
  --grid.y-number 4 \
  --grid.x-length 250 \
  --grid.y-length 250 \
  --default-lanenumber 2 \
  -o manhattan.net.xml


# ===============================
# 2) Generate Vehicle Trips
# ===============================
python3 $SUMO_HOME/tools/randomTrips.py \
  -n manhattan.net.xml \
  -o trips.trips.xml \
  --begin 0 \
  --end 300 \
  --period 1 \
  --trip-attributes='departPos="random" departSpeed="max"' \
  --seed 42


# ===============================
# 3) Generate Routes
# ===============================
duarouter \
  -n manhattan.net.xml \
  -t trips.trips.xml \
  -o routes.rou.xml


# ===============================
# 4) RSU Deployment File
# ===============================
cat << EOF > rsu.add.xml
<additional>
    <poi id="RSU1" x="250" y="250" color="1,0,0"/>
    <poi id="RSU2" x="500" y="250" color="1,0,0"/>
    <poi id="RSU3" x="250" y="500" color="1,0,0"/>
    <poi id="RSU4" x="500" y="500" color="1,0,0"/>
    <poi id="RSU5" x="750" y="250" color="1,0,0"/>
    <poi id="RSU6" x="250" y="750" color="1,0,0"/>
    <poi id="RSU7" x="500" y="750" color="1,0,0"/>
    <poi id="RSU8" x="750" y="500" color="1,0,0"/>
    <poi id="RSU9" x="750" y="750" color="1,0,0"/>
</additional>
EOF


# ===============================
# 5) SUMO Configuration File
# ===============================
cat << EOF > manhattan.sumocfg
<configuration>
    <input>
        <net-file value="manhattan.net.xml"/>
        <route-files value="routes.rou.xml"/>
        <additional-files value="rsu.add.xml"/>
    </input>

    <time>
        <begin value="0"/>
        <end value="300"/>
    </time>

    <gui_only>
        <start value="true"/>
    </gui_only>
</configuration>
EOF


# ===============================
# 6) Run SUMO GUI
# ===============================
sumo-gui -c manhattan.sumocfg
