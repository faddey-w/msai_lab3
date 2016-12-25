_={
    scene: {
        x: 0,
        y: 0,
        width: 400,
        height: 300,
        obstacle: {
            hw0: {
                color: 0xff000000,
                points: [
                    {x:   0, y:  99},
                    {x: 100, y:  99},
                    {x: 100, y: 101},
                    {x:   0, y: 101},
                ]
            },
            hw1: {
                color: 0xff000000,
                points: [
                    {x: 145, y:  99},
                    {x: 200, y:  99},
                    {x: 200, y: 101},
                    {x: 145, y: 101},
                ]
            },
            vw0: {
                color: 0xff000000,
                points: [
                    {x: 170, y:  0},
                    {x: 172, y:  0},
                    {x: 172, y: 100},
                    {x: 170, y: 100},
                ]
            },
            hw2: {
                color: 0xff000000,
                points: [
                    {x: 250, y:  99},
                    {x: 310, y:  99},
                    {x: 310, y: 101},
                    {x: 250, y: 101},
                ]
            },
            vw1: {
                color: 0xff000000,
                points: [
                    {x: 280, y:  0},
                    {x: 282, y:  0},
                    {x: 282, y: 100},
                    {x: 280, y: 100},
                ]
            },
            hw3: {
                color: 0xff000000,
                points: [
                    {x: 360, y:  99},
                    {x: 400, y:  99},
                    {x: 400, y: 101},
                    {x: 360, y: 101},
                ]
            },
            hu0: {
                color: 0xff000000,
                points: [
                    {x:   0, y: 199},
                    {x: 50, y: 199},
                    {x: 50, y: 201},
                    {x:   0, y: 201},
                ]
            },
            hu1: {
                color: 0xff000000,
                points: [
                    {x: 100, y: 199},
                    {x: 170, y: 199},
                    {x: 170, y: 201},
                    {x: 100, y: 201},
                ]
            },
            vu0: {
                color: 0xff000000,
                points: [
                    {x: 120, y: 200},
                    {x: 122, y: 200},
                    {x: 122, y: 300},
                    {x: 120, y: 300},
                ]
            },
            hu2: {
                color: 0xff000000,
                points: [
                    {x: 230, y: 199},
                    {x: 280, y: 199},
                    {x: 280, y: 201},
                    {x: 230, y: 201},
                ]
            },
            vu1: {
                color: 0xff000000,
                points: [
                    {x: 250, y: 200},
                    {x: 252, y: 200},
                    {x: 252, y: 300},
                    {x: 250, y: 300},
                ]
            },
            hu3: {
                color: 0xff000000,
                points: [
                    {x: 330, y: 199},
                    {x: 400, y: 199},
                    {x: 400, y: 201},
                    {x: 330, y: 201},
                ]
            },
        },
        color: 0xffffffff,
        colorArea: {
        },
        slope: 0,
        slopeArea: {
        }
    },

    net: {
        port: 20037
    },

    robot: {
        arduino: {
            x: 20,
            y: 150,
            phi: 0,
            radius: 15,
            v: 0,
            omega: 0,
            wheelDFC: 7.72172260574025,
            wheelPhi: 1.06369782240256,
            sensor: [
                {type: "ultrasonic", phi: 0, rphi: 0, r: 0, a: 0},
                {type: "floor-light", phi: Math.PI / 9, r: 13, a: 0},
                {type: "floor-light", phi: 0, r: 13, a: 0},
                {type: "floor-light", phi: -Math.PI / 9, r: 13, a: 0},
                {type: "gyroscope", axis: "x", a: 0},
                {type: "gyroscope", axis: "y", a: 0},
                {type: "gyroscope", axis: "z", a: 0},
                {type: "microphone", a: 0},
                {type: "touch", phi: 0, a: 0},
                {type: "touch", phi: Math.PI, a: 0},
            ],
            motor: [0, 0, 0, -1, -1, -1],
            motorOmega: 17.6,
            battery: {
                value: 9,
                min: 6,
                max: 10
            },
            manualControl: true,

            wheelRadius: 2.5,
	    
	    rotationKoeff: 0.46,

            init: function () { },
            adjust: function (dt) {
                this.v = (this.motor[0] + this.motor[1]) * this.motorOmega * this.wheelRadius / 2;
                this.omega = (this.motor[1] - this.motor[0]) * this.motorOmega * this.wheelRadius * Math.sin(this.wheelPhi) * this.rotationKoeff / (2 * this.wheelDFC);
                this.sensor[0].phi = this.motor[2] * Math.PI / 1.80;//+= dt * this.motorOmega * this.motor[2];
                /*if (this.sensor[2].phi < 0)
                    this.sensor[2].phi += Math.PI;
                if (this.sensor[2].phi >= 2 * Math.PI)
                    this.sensor[2].phi -= Math.PI;*/
                //this.motive.MP.a = Math.max(this.sensor[0].a, this.sensor[1].a);
                //this.motive.MC.a = 2 * (this.battery.max - this.battery.value) / (this.battery.max - this.battery.min) - 1;
                this.hardcoded_doors();
            },

            hardcoded_doors: function() {
                if (this.y > 150 + this.radius + 3) // at least #1
                    _.scene.obstacle.door01.active = true;
            }
        }
    }
}
