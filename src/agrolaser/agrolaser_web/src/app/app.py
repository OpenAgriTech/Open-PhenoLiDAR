import pathlib
import os
import shutil
from flask import Flask
import logging
import dash
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_daq as daq
import dash_html_components as html
import numpy as np
import plotly.graph_objs as go
import roslibpy
import time
from dash.dependencies import State, Input, Output

import pointcloud_msg

server = Flask(__name__)
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = dash.Dash(
    __name__, external_stylesheets=[dbc.themes.SLATE],
    meta_tags=[
        {"name": "viewport", "content": "width=device-width, initial-scale=1.0"}
    ],
    title="Open AgroLiDAR Control",
    server=server
)

# This is for gunicorn
#server = app.server

# Mapbox
MAPBOX_ACCESS_TOKEN = "pk.eyJ1IjoiamFqYmVybmkiLCJhIjoiY2oyMXFsZjdsMDAxNTJybzd0bDNxczZyeCJ9.6EKxvkWLdnzNI0RJLAsimA"
MAPBOX_STYLE = "mapbox://styles/jajberni/ckk35n9qg3uvw17qg3du25oyb"

GPS_FIX_COLORS = {6: "#67d03b", 5: "#f9f025", 4: "#f97654", 3: "#f97654", 2: "#f43c4e", 1: "#f43c4e", 0: "#f43c4e",
                  -1: "#f43c4e"}

DEFAULT_FOLDER = "/data"
class StatusManager:
    """Class to store information useful to callbacks"""

    def __init__(self):
        self.is_connected = False
        self.scan_count = 0
        self.lat = 0.0
        self.lon = 0.0
        self.h_accuracy = 0.0
        self.v_accuracy = 0.0
        self.gps_status = -1
        self.lat_path = []
        self.lon_path = []
        self.fix_path = []
        self.speed_kph = 0.0
        self.last_cloud = None
        self.last_gps = None
        self.last_pose = None
        self.last_sat_count = 0
        self.rtk_listener = None
        self.cloud_listener = None
        self.pose_listener = None
        self.project_talker = None
        self.project_service = None
        self.project_list = []

        ros_master = '127.0.0.1'
        if "ROS_MASTER_HOSTNAME" in os.environ:
            ros_master = os.environ["ROS_MASTER_HOSTNAME"]

        self.client = roslibpy.Ros(host=ros_master, port=9090)
        self.connect()

    def connect(self):
        try:
            self.client.run(timeout=50)
            self.is_connected = True
            self.create_listeners()
        except Exception as ex:
            self.is_connected = False
            print("Error connecting to ROS")

    def rtk_callback(self, msg):
        self.last_gps = msg
        self.lat = msg['lat'] /1e7
        self.lon = msg['lon'] / 1e7
        self.h_accuracy = msg['h_acc']/1e3
        self.v_accuracy = msg['v_acc']/1e3
        self.gps_status = msg['fix_type']
        self.lat_path.append(self.lat)
        self.lon_path.append(self.lon)
        self.fix_path.append(self.gps_status)
        self.speed_kph = msg['vel']*0.036
        self.last_sat_count = msg['satellites_visible']
        #print(msg['cog']/100, msg['fix_type'], msg['dgps_age'])

    def cloud_callback(self, msg):
        self.scan_count += 1
        self.last_cloud = pointcloud_msg.msg_to_cloud(msg)

    def pose_callback(self, msg):
        self.last_pose = msg

    def start_recording(self, project_name):
        if self.is_connected:
            self.project_talker.publish(roslibpy.Message({'data': project_name}))

    def stop_recording(self):
        if self.is_connected:
            self.project_talker.publish(roslibpy.Message({'data': '.'}))

    def create_listeners(self):
        if self.is_connected:
            self.project_talker = roslibpy.Topic(self.client, '/project/name', 'std_msgs/String')

            self.rtk_listener = roslibpy.Topic(self.client, '/mavros/gpsstatus/gps1/raw', 'mavros_msgs/GPSRAW')
            self.rtk_listener.subscribe(self.rtk_callback)

            self.cloud_listener = roslibpy.Topic(self.client, '/laserMapping/laser_points', 'sensor_msgs/PointCloud2')
            self.cloud_listener.subscribe(self.cloud_callback)

            self.pose_listener = roslibpy.Topic(self.client, '/mavros/global_position/local', 'nav_msgs/Odometry')
            self.pose_listener.subscribe(self.pose_callback)

            self.project_service = roslibpy.Service(self.client, '/project_service', 'agrolaser_node/ProjectService')
            project_request = roslibpy.ServiceRequest({'request_string': 'list', 'project': ''})
            self.project_list = [{'label': project_name, 'value': project_name} for project_name in sorted(self.project_service.call(project_request)['list_strings'])]
            if len(self.project_list) == 0:
                self.project_list = [{'label': 'Test', 'value': 'Test'}]


local_vars = StatusManager()

# Point Cloud graph components
# Helix equation for demo
t = np.linspace(0, 10, 50)
x, y, z = np.cos(t), np.sin(t), t

default_fig = go.Figure(data=[go.Scatter3d(x=x, y=y, z=z,
                                           mode='markers',
                                           marker=dict(
                                               size=1,
                                               opacity=0.8
                                           ))])

default_fig.update_layout(hovermode=False)
default_fig.update_layout(margin=dict(l=0, r=0, b=0, t=0))
default_fig.update_scenes(aspectmode='manual', aspectratio_z=0.1)

cloud_graph_card = dbc.Card(
    dcc.Graph(id='point-cloud-graph', figure=default_fig, config={
        'displayModeBar': False,
    }), body=True
)

update_button = dbc.Button(
    "Clear Point Cloud", id="update", n_clicks=0, color="primary", outline=True
)

update_button_2 = dbc.Button(
    "Force Update", id="update-2", n_clicks=0, color="primary", outline=True
)

setup_button = html.Div(
    [
        dbc.Button("Options", id="open-setup"),
        dbc.Modal(
            [
                dbc.ModalHeader("Options"),
                dbc.ModalBody("This is the content of the modal"),
                dbc.ModalFooter(
                    dbc.Button("Close", id="close-setup", className="ml-auto")
                ),
            ],
            id="setup-modal",
        ),
    ]
)

new_project_button = html.Div(
    [
        dbc.Button("New Project", id="new-project-button"),
        dbc.Modal(
            [dbc.Row(
                dbc.Col(dbc.FormGroup(
                            [
                                dbc.Label("Project Name", className="mr-2"),
                                dbc.Input(id="input-new-project-name", type="text", placeholder="Enter project"),
                            ],
                            className="mr-3",
                        )), form=True),
            dbc.Row(
                dbc.Col(dbc.FormGroup(
                            [
                                dbc.Label("Description", className="mr-2"),
                                dbc.Input(type="text", placeholder="Enter description"),
                            ],
                            className="mr-3",
                        )), form=True),
            dbc.Row([
                        dbc.Col(dbc.Button("Accept", color="primary", id="accept-project-button")),
                        dbc.Col(dbc.Button("Cancel", color="primary", id="cancel-project-button")),
                    ],
                )
            ],
            id="new-project-modal",
        ),
    ]
)

# Dash_DAQ elements

utc = html.Div(
    id="control-panel-utc",
    children=[
        daq.LEDDisplay(
            id="control-panel-utc-component",
            value="16:23",
            label="Time",
            size=40,
            color="#fec036",
            backgroundColor="#2b2b2b",
        )
    ],
    n_clicks=0,
)

speed = daq.Gauge(
    id="control-panel-speed-component",
    label="Speed",
    min=0,
    max=10,
    showCurrentValue=True,
    value=4.0,
    size=175,
    units="km/h",
    color="#fec036",
)

scan_count = daq.LEDDisplay(
    id="control-panel-scans-component",
    value="0000000",
    label="Scans",
    size=24,
    color="#fec036",
    style={"color": "#black"},
    backgroundColor="#2b2b2b",
)

storage_indicator = html.Div(
    id="control-panel-disk",
    children=[
        daq.GraduatedBar(
            id="control-panel-disk-component",
            label="Disk Capacity",
            min=0,
            max=100,
            value=76,
            step=1,
            showCurrentValue=True,
            color="#fec036",
        )
    ],
    n_clicks=0,
)

battery_indicator = html.Div(
    id="control-panel-battery",
    children=[
        daq.GraduatedBar(
            id="control-panel-battery-component",
            label="Battery-Level",
            min=0,
            max=100,
            value=85,
            step=1,
            showCurrentValue=True,
            color="#fec036",
        )
    ],
    n_clicks=0,
)

longitude = daq.LEDDisplay(
    id="control-panel-longitude-component",
    value="0000.0000",
    label="Longitude",
    size=24,
    color="#fec036",
    style={"color": "#black"},
    backgroundColor="#2b2b2b",
)

latitude = daq.LEDDisplay(
    id="control-panel-latitude-component",
    value="0050.9789",
    label="Latitude",
    size=24,
    color="#fec036",
    style={"color": "#black"},
    backgroundColor="#2b2b2b",
)

h_accuracy = daq.LEDDisplay(
    id="control-panel-h-accuracy-component",
    value="0.0000",
    label="H Accuracy (m)",
    size=24,
    color="#fec036",
    style={"color": "#black"},
    backgroundColor="#2b2b2b",
)

v_accuracy = daq.LEDDisplay(
    id="control-panel-v-accuracy-component",
    value="0.0000",
    label="V Accuracy (m)",
    size=24,
    color="#fec036",
    style={"color": "#black"},
    backgroundColor="#2b2b2b",
)

satellites = html.Div([
    dbc.Row([
        dbc.Col(
            daq.LEDDisplay(
                id="satellite-count",
                value="00",
                label="Satellites",
                size=24,
                color="#fec036",
                style={"color": "#black"},
                backgroundColor="#2b2b2b",
            )
        ),
        dbc.Col(
            daq.Indicator(
                id="rtk-indicator",
                label="RTK Status",
                labelPosition="bottom",
                value=True,
                color="#15e82e",
                style={"color": "#black"},
            )
        ),

    ], no_gutters=True, align="center")
])

gps_card = dbc.Card([
    satellites,
    dbc.Row([
        dbc.Col([
            latitude,
            longitude]),
        dbc.Col([
            h_accuracy,
            v_accuracy]),
    ])
])


map_toggle = daq.ToggleSwitch(
    id="control-panel-toggle-map",
    value=True,
    label=["Hide path", "Show path"],
    color="#ffe102",
    style={"color": "#black"},
)

# Side panel

project_dropdown_text = html.P(
    id="project-dropdown-text", children=["Control"]
)
"""project_dropdown = dbc.FormGroup(
        [
            dbc.Label("Camera Position"),
            dbc.Select(
                id="project",
                options=[
                    {"label": "New Project...", "value": "new_project"},
                    {"label": "Project 1", "value": "project_1"},
                    {"label": "Project 2", "value": "project_2"},
                ],
                value="project_1",
            ),
        ]
    )
"""

project_select = dbc.InputGroup(
    [
        dbc.InputGroupAddon("Select Project", addon_type="prepend"),
        dbc.Select(
            id="project-dropdown-component",
            options=local_vars.project_list,
            value=local_vars.project_list[0]['value']
        ),
        dbc.InputGroupAddon(
            new_project_button,
            addon_type="append",
        ),
    ]
),

project_title = html.H1(id="project-name", children="")

recording_button = daq.PowerButton(
    id='recording-button',
    on=False,
    color='#FF5E5E',
    size=80,
    label='Record',
    labelPosition='top'
)

project_body = html.P(
    className="project-description", id="project-description", children=[""]
)

side_panel_layout = html.Div(
    id="panel-side",
    children=[
        dbc.Card([
            dbc.Row([
                dbc.Col(project_select),
            ]),
            dbc.Row([
                dbc.Col(recording_button),
            ]),
            dbc.Row([
                dbc.Col(update_button),
                dbc.Col(update_button_2),
                dbc.Col(setup_button),
            ])
        ]),
    ],
)


# project location tracker

# Helper to straighten lines on the map
def flatten_path(xy1, xy2):
    diff_rate = (xy2 - xy1) / 100
    res_list = []
    for i in range(100):
        res_list.append(xy1 + i * diff_rate)
    return res_list


map_data = [
    {
        "type": "scattermapbox",
        "lat": [0],
        "lon": [0],
        "hoverinfo": "text+lon+lat",
        "text": "LiDAR Path",
        "mode": "lines",
        "line": {"width": 3, "color": "#126de3"},
    },
    {
        "type": "scattermapbox",
        "lat": [0],
        "lon": [0],
        "hoverinfo": "text+lon+lat",
        "text": "Current Position",
        "mode": "markers",
        "marker": {"size": 10, "color": "#fec036"},
    },
]

map_layout = {
    "mapbox": {
        "accesstoken": MAPBOX_ACCESS_TOKEN,
        "style": MAPBOX_STYLE,
        "center": {"lat": 37.8, "lon": -4.8}, "zoom": 16,
    },
    "showlegend": False,
    "autosize": True,
    "paper_bgcolor": "#1e1e1e",
    "plot_bgcolor": "#1e1e1e",
    "margin": {"t": 0, "r": 0, "b": 0, "l": 0},
}

map_graph = dbc.Card(
    id="world-map-wrapper",
    children=[
        map_toggle,
        dcc.Graph(
            id="world-map",
            figure={"data": map_data, "layout": map_layout},
            config={"displayModeBar": False, "scrollZoom": True},
        )
    ],
    body=True
)

main_panel_card = html.Div([
    dcc.Interval(id="interval", interval=1 * 2000, n_intervals=0),
    dcc.Interval(id="interval-fast", interval=500, n_intervals=0),
    dbc.Card([
        dbc.Row([
            dbc.Col(speed, width=3),
            dbc.Col([dbc.Row(utc), dbc.Row(scan_count), dbc.Row(storage_indicator)], width=3),
            dbc.Col(gps_card, width=5)
        ]
        ),
    ]),
    dbc.Card(dbc.Row([dbc.Col(cloud_graph_card, width=6), dbc.Col(map_graph, width=6)]))
])

# Data generation

# Pandas
APP_PATH = str(pathlib.Path(__file__).parent.resolve())

# Root
root_layout = dbc.Container(
    [
        dcc.Store(id="store-placeholder"),
        dcc.Store(
            id="store-data"),
        html.H1("Open PhenoLiDAR Control"),
        html.Hr(),
        dbc.Row([
            dbc.Col(main_panel_card, md=8),
            dbc.Col(side_panel_layout, md=4),
        ], align="start")
    ], fluid=True,
)

app.layout = root_layout


# Callback free space
@app.callback(
    Output("control-panel-disk-component", "value"), [Input("interval", "n_intervals")]
)
def update_free_disk(interval):
    total, used, free = shutil.disk_usage("/")
    free_pc = 100 * used / total
    return free_pc


# Callbacks Data


# Callbacks Components


@app.callback(
    Output("control-panel-utc-component", "value"), [Input("interval", "n_intervals")]
)
def update_time(interval):
    hour = time.localtime(time.time())[3]
    hour = str(hour).zfill(2)

    minute = time.localtime(time.time())[4]
    minute = str(minute).zfill(2)
    return hour + ":" + minute


@app.callback(
    [
        Output("control-panel-latitude-component", "value"),
        Output("control-panel-longitude-component", "value"),
        Output("control-panel-h-accuracy-component", "value"),
        Output("control-panel-v-accuracy-component", "value"),
        Output("control-panel-scans-component", "value"),
        Output("satellite-count", "value"),
        Output("rtk-indicator", "value"),
        Output("rtk-indicator", "color"),
    ],
    [Input("interval", "n_intervals")],
)
def update_gps_component(clicks):
    rtk_status = False
    rtk_color = "#fec036"
    if local_vars.gps_status > 5:
        rtk_status = True
    if local_vars.gps_status < 3:
        rtk_color = "#dc1330"

    elif local_vars.gps_status == 5:
        rtk_color = "#f9f025"
    elif local_vars.gps_status == 6:
        rtk_color = "#6bd71f"
    else:
        rtk_color = "#dc1330"
    return "{:.4f}".format(local_vars.lat), "{:.4f}".format(
        local_vars.lon), "{:.3f}".format(local_vars.h_accuracy), "{:.3f}".format(
        local_vars.v_accuracy),  "{:08d}".format(local_vars.scan_count), local_vars.last_sat_count, rtk_status, rtk_color


@app.callback(Output("control-panel-speed-component", "value"),
              [Input("interval-fast", "n_intervals")],
              )
def update_speed_component(clicks):
    return local_vars.speed_kph


@app.callback(
    Output("point-cloud-graph", "figure"),
    [Input("update", "n_clicks"), ],
    [State("point-cloud-graph", "figure")]
)
def create_cloud_graph(clicks, graph_data):
    if local_vars.last_cloud is not None:
        # print(graph_data)
        # print(local_vars.last_cloud.points.head())
        df = local_vars.last_cloud
        graph_data['data'] = [
            go.Scatter3d(
                x=df['x'],
                y=df['y'],
                z=df['z'],
                mode='markers',
                marker=dict(
                    size=1,
                    color=df['intensity'],
                    opacity=0.8
                )
            )
        ]

    else:
        print("No data")
    return graph_data


@app.callback(
    Output("point-cloud-graph", "extendData"),
    [Input("interval", "n_intervals"), Input("update-2", "n_clicks"), ],
    [State("point-cloud-graph", "figure")]
)
def update_cloud_graph(interval, clicks, graph_data):
    # print(graph_data['data'])

    if local_vars.last_cloud is not None:

        df = local_vars.last_cloud
        data = [go.Scatter3d(
            x=df['x'],
            y=df['y'],
            z=df['z'],
            mode='markers',
            marker=dict(
                size=1,
                color=df['intensity'],
                opacity=0.8
            )
        )]
        # print(data[0]['marker'])
        if graph_data is None:
            return
        if len(graph_data['data']) > 0:
            # return data[0], [0]
            return dict(x=[data[0]['x']], y=[data[0]['y']], z=[data[0]['z']]), [
                0]  # , marker=dict(color=[data[0]['marker']['color']])), [0]
        # return data


@app.callback(
    Output("world-map", "figure"),
    [
        Input("interval", "n_intervals"),
        Input("control-panel-toggle-map", "value"),
    ],
    [
        State("world-map", "figure"),
        State("store-data", "data"),
    ],
)
def update_word_map(clicks, toggle, old_figure, data):
    figure = old_figure

    figure["data"][1]["lat"] = [local_vars.lat]
    figure["data"][1]["lon"] = [local_vars.lon]
    figure["data"][1]["marker"]["color"] = GPS_FIX_COLORS[local_vars.gps_status]

    figure["layout"]["mapbox"]["center"] = {"lat": local_vars.lat, "lon": local_vars.lon}

    if not toggle:
        figure["data"][0]["lat"] = []
        figure["data"][0]["lon"] = []
    else:
        figure["data"][0]["lat"] = local_vars.lat_path
        figure["data"][0]["lon"] = local_vars.lon_path

    return figure


@app.callback(
    [Output("project-dropdown-component", "disabled"), Output("new-project-button", "disabled")],
    [
        Input("recording-button", "on"),
        Input("project-dropdown-component", "value"),
    ],
)
def recording_control(on, project):
    if project != 'new_project':
        if on:
            local_vars.start_recording(project)
        else:
            local_vars.stop_recording()
    return on, on


@app.callback(
    Output("setup-modal", "is_open"),
    [Input("open-setup", "n_clicks"), Input("close-setup", "n_clicks")],
    [State("setup-modal", "is_open")],
)
def toggle_modal(n1, n2, is_open):
    if n1 or n2:
        return not is_open
    return is_open

@app.callback(
    [Output("new-project-modal", "is_open"), Output("project-dropdown-component", "options")],
    [Input("new-project-button", "n_clicks"), Input("accept-project-button", "n_clicks"),
     Input("cancel-project-button", "n_clicks"), Input("input-new-project-name", "value")],
    [State("new-project-modal", "is_open"), State("accept-project-button", "n_clicks"), State("cancel-project-button", "n_clicks")],
)
def toggle_modal(n1, n2, n3, new_project_name, is_open, n2_s, n3_s):
    if n1 is None:
        return is_open, local_vars.project_list
    if n2 == n1:
        print("Create new project: " + new_project_name)
        resp = local_vars.project_service.call(roslibpy.ServiceRequest({'request_string': 'create', 'project': new_project_name}))
        project_request = roslibpy.ServiceRequest({'request_string': 'list', 'project': ''})
        local_vars.project_list = [{'label': project_name, 'value': project_name} for project_name in
                             sorted(local_vars.project_service.call(project_request)['list_strings'])]

        return False, local_vars.project_list
    if n3 == n1:
        return False, local_vars.project_list
    if n1:
        return True, local_vars.project_list
    return is_open, local_vars.project_list


if __name__ == "__main__":
    debug = True
    port = 8051
    if "DASH_DEBUG_MODE" in os.environ:
        debug = False if os.environ["DASH_DEBUG_MODE"] == "False" else True
    if "DASH_PORT" in os.environ:
        port = os.environ["DASH_PORT"]
    app.run_server(host="0.0.0.0", port=port, debug=debug)
