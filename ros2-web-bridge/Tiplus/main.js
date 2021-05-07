var app = new Vue({
        el: '#app',
        // storing the state of the page
        data: {
            connected: false,
            ros: null,
            logs: [],
            loading: false,
            rosbridge_address: 'localhost',
            port: '9090',
            mapViewer: null,
            mapGridClient: null,
            interval: null,
        },
        // helper methods to connect to ROS
        methods: {
            connect: function() {
                this.loading = true
                this.ros = new ROSLIB.Ros({
                    url: this.rosbridge_address
                })
                 this.ros.on('connection', () => {
                    this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                    this.connected = true
                    this.loading = false
    
                    this.mapViewer = new ROS2D.Viewer({
                        divID: 'map',
                        width: 420,
                        height: 360
                    })
    
                    // Setup the map client.
                    this.mapGridClient = new ROS2D.OccupancyGridClient({
                        ros: this.ros,
                        rootObject: this.mapViewer.scene,
                        continuous: true,
                    })
                    // Scale the canvas to fit to the map
                    this.mapGridClient.on('change', () => {
                        this.mapViewer.scaleToDimensions(this.mapGridClient.currentGrid.width, this.mapGridClient.currentGrid.height);
                        this.mapViewer.shift(this.mapGridClient.currentGrid.pose.position.x, this.mapGridClient.currentGrid.pose.position.y)
                    })
    
                })
                this.ros.on('error', (error) => {
                    this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
                })
                this.ros.on('close', () => {
                    this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                    this.connected = false
                    this.loading = false
                    document.getElementById('map').innerHTML = ''
                })
            },
            disconnect: function() {
                this.ros.close()
            },
        },
        mounted() {
            this.interval = setInterval(() => {
                if (this.ros != null && this.ros.isConnected) {
                    this.ros.getNodes((data) => { }, (error) => { })
                }
            }, 10000)
        },
    })
    