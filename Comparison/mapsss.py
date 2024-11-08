import folium
import webbrowser

# List of coordinates (latitude, longitude) for multiple nodes
arcs = {'SB1': [34.06748049349774, -117.58627467784373, 34.06733829280365, -117.56805711511826, 2],
        'SB2': [34.073664262003916, -117.54478073959717, 34.07880072708419, -117.54469490891294, 3],
        'SB3': [34.055221667702725, -117.54707552414482, 34.042942672497496, -117.55046155255096, 2],
        'SB4': [34.13509556151658, -117.5918373464612, 34.13665850403775, -117.55952209385272, 3],
        'LA1': [34.068996100878536, -118.23639721457458, 34.08111761574252, -118.22502464857632, 5],
        'LA2': [34.00204584929778, -118.28106671082655, 34.02099821349527, -118.27820873785127, 4],
        'LA3': [34.1820571382235, -118.47097754846425, 34.238471386274036, -118.47316244104785, 5],
        'LA4': [34.01006543492014, -118.4164022951023, 34.034039070699464, -118.43717332130281, 4],
        'LA5': [33.98578244932628, -118.39797964305346, 33.970081316208265, -118.37465654195792, 5],
        'LA6': [34.06144667253944, -118.2153262230426, 34.0759258090762, -118.21921047652454, 4],
        'LA7': [33.99983225156185, -118.14864437645414, 33.978268943166086, -118.1272725354462, 4],
        'LA8': [33.96772442600475, -118.08341546454861, 33.93582806531739, -118.09937997228951, 4],
        'RS1': [33.94245747674634, -117.27950645054175, 33.94101794657498, -117.2530191660786, 3],
        'RS2': [34.01310741213049, -117.44103478257155, 34.01129317274416, -117.43197964538643, 2],
        'RS3': [33.88454308524973, -117.62931466068288, 33.88289535845872, -117.644110477192, 2],
        'RS4': [33.897340150833756, -117.48842212774255, 33.894628991533885, -117.50064506537053, 2]}

fixed_points = {'DP': [34.02889809043227, -117.83417609430023],
                'RS_iDL': [33.93341044928401, -117.45850404694579],
                'SB_iDL': [34.082150272643005, -117.56158799481022],
                'LA_iDL': [34.03829254642072, -118.28026863444143]}

coordinates = list(arcs.values())

midpoint = [(sum(i[0]+i[2] for i in coordinates) / (2*len(coordinates))),
            (sum(i[1]+i[3] for i in coordinates) / (2*len(coordinates)))]
my_map = folium.Map(location=midpoint, zoom_start=10)

for i, coord in fixed_points.items():
    folium.Marker(location=(coord[0], coord[1]), color='blue', tooltip=folium.Tooltip(str(i)+' ('+str(coord[0])+','+str(coord[1])+')')).add_to(my_map)
    # folium.Marker(location=(coord[0], coord[1]), icon=folium.DivIcon(html=f"""<div style="font-size: 15px; color: red">{str(i)}</div>""")).add_to(my_map)


for i, coord in arcs.items():
    folium.CircleMarker(location=(coord[0], coord[1]), radius=3, color='blue', fill=True, fill_color='blue', fill_opacity=0.7, tooltip=folium.Tooltip(str(i)+' ('+str(coord[0])+','+str(coord[1])+')')).add_to(my_map)
    folium.CircleMarker(location=(coord[2], coord[3]), radius=3, color='red', fill=True, fill_color='red', fill_opacity=0.7, tooltip=folium.Tooltip(str(i)+' ('+str(coord[2])+','+str(coord[3])+')')).add_to(my_map)

# Draw separate lines between each consecutive pair of points
for j,i in enumerate(coordinates):
    start_coord = (i[0], i[1])
    end_coord = (i[2], i[3])

    # Add line between start and end coordinates
    if j < 4:
        folium.PolyLine([start_coord, end_coord], color="black", weight=2.5, opacity=1).add_to(my_map)
    elif 4 <= j <= 11:
        folium.PolyLine([start_coord, end_coord], color="red", weight=2.5, opacity=1).add_to(my_map)
    else:
        folium.PolyLine([start_coord, end_coord], color="green", weight=2.5, opacity=1).add_to(my_map)

map_path = "map_with_separate_lines.html"
my_map.save(map_path)

webbrowser.open(map_path)


