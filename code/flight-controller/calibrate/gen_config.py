import json 

data = {
    'ahrs': {
        'gain_normal': 0.5,
        'gain_init': 10.0,
        'init_time': 3.0,

        'min_mag_distortion': 0.22,
        'max_mag_distoriton': 0.67,
        'declination': 15.2,
        'add_declination': False,

        'accel_rejection': 0.1,
        'accel_rejection_t': 0.1,

        'gyro_calibrate': {
            'bias': [0.0, 0.0, 0.0],
            'sensitivity': [1.0, 1.0, 1.0],
            'rotation_matrix': [[1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0],],
        },
    'accel_calibrate': {
            'bias': [0.0, 0.0, 0.0],
            'sensitivity': [1.0, 1.0, 1.0],
            'rotation_matrix': [[1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0],],
        },
    'mag_calibrate': {
            'hard_iorn': [0.0, 0.0, 0.0],
            'soft_iorn': [[1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0],
                          [0.0, 0.0, 1.0],],
            'rotation_matrix': [[1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0],],
        },
    },
    'imu': {
        'accel_scale': 2,
        'gyro_scale': 250,
        'mag_scale': 4,

        'data_rate': 6,
        'mag_data_rate': 7,

        'mag_mode': 3,
        'mag_temp_comp': 0,
    },
    'motors': {
        'front_left': {
            'pin': 12,
            'min': 1000,
            'max': 2000,
        },
        'front_right': {
            'pin': 13,
            'min': 1000,
            'max': 2000,
        },
        'back_left': {
            'pin': 20,
            'min': 1000,
            'max': 2000,
        },
        'back_right': {
            'pin': 21,
            'min': 1000,
            'max': 2000,
        },

    },
    "pid": {
        "x": {
            "kP": 1,
            "kI": 0,
            "kD": 0,
            "gain": 1,
        },
        "y": {
            "kP": 1,
            "kI": 0,
            "kD": 0,
            "gain": 1,
        },
        "z": {
            "kP": 0.1,
            "kI": 0,
            "kD": 0,
            "gain": 1,
        },
    },
}

if input('Warning! Will rewrite config file with default values! Enter y to continue. ') != 'y': 
    exit()

with open('../config.json', 'w', encoding='utf-8') as file:
    json.dump(data, file, indent=4)

