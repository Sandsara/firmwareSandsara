# BLE Communication {#BLECommunication}
If you are interested in creating your own app to communicate to Sandara you can find all you need to do it in our documentation. Here we explain how to read the doc.

## BLE Basics
We use Bluetooth Low Energy (BLE) to communicate to Sandsara and change some of its properties. First of all, we recommend reading some basics about BLE, here you can see some links you can read
- https://www.bluetooth.com/learn-about-bluetooth/tech-overview/
- https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/

We use the esp32 as a BLE server and you can connect to it as a client.

## Services and Characteristics
We structure the comunication using 4 services and several characteristics. the services are list below:
|   | name             | description                                     | UUID                                 |
|---|------------------|-------------------------------------------------|--------------------------------------|
| 1 | General Service  | it is used to change general properties         | fd31a840-22e7-11eb-adc1-0242ac120002 |
| 2 | File Service     | It is used to receive or send files             | fd31abc4-22e7-11eb-adc1-0242ac120002 |
| 3 | Playlist Service | it is used to control the playback of the paths | fd31a778-22e7-11eb-adc1-0242ac120002 |
| 4 | LedStrip Service | It is used to control de led strip              | fd31a2be-22e7-11eb-adc1-0242ac120002 |

As you know every service or characteristic can be indentified by its UUID so you can see above. In every service you will find its characteristics related to the description.
## BLE commands
In the documentation, we use the characteristics and commands words interchangeably and we describe every characteristic by its class. In order to make things clear let us explain how a characteristic is created in ESP32.
1. we create the BLECharacteristic object.
2. we use this object to handle the characteristic
3. we create a class that inherits the BLECharacteristicCallbacks class.
4. we associate the BLECharacteristic object to a BLECharacteristicCallbacks object so we can handle it every time the characteristic is written or read.

For explanatory purposes, we treat each BLECharacteristicCallbacks class as a characteristic (BLE command) so you will find all the characteristics on the <a href="annotated.html">Class list</a> page. Every class has a descriptive name so you can infer what is the purpose of each characteristic.

For example, let's take the generalCallbacks_speed class, you can infer that this characteristic is used to modify the speed of Sandsara, in any way when you open the class you will see a description and how to use it. There are some things that are in each command, you can see a description of each section in a command page.

### Description
At the top, you will find a description of the characteristic.

### properties
You will find a section called properties where you can find the properties of each characteristic, the possible properties are listed below:
- **read** means you can read the characteristic.
- **write** means you can write to the characteristic.
- **notify** you will be able to receive notifications from the characteristic.

### usage
In this section, you can see how to use the characteristic

### Range
Here you find what are the values the characteristic accepts

### BLE Error Messeages
In order to give you feedback about your interaction with the devices, every service has a characteristic that notifies you of the response of your interaction with the characteristics, for example, if you set a new speed you may need a confirmation if the speed was set or not. That is the reason why we create an errorMsg characteristic in each service. We recommend following the next steps before you communicate with BLE.
1. Identify the service of the characteristic you will interact with..
2. activate the errorMsg characteristic of that service.
3. interact with the desired characteristic.
4. wait to be notified by the errorMsg characteristic and read it for the response.

Here you can see the UUID of each erroMsg Service characteristic.
|   | Service          | errorMsg characteristic | UUID                                 |
|---|------------------|-------------------------|--------------------------------------|
| 1 | General Service  | generalErrorMsg         | 7b204d4a-30c3-11eb-adc1-0242ac120002 |
| 2 | File Service     | fileErrorMsg            | fcbffce2-2af1-11eb-adc1-0242ac120002 |
| 3 | Playlist Service | playlistErrorMsg        | 9b12aa02-2c6e-11eb-adc1-0242ac120002 |
| 4 | LedStrip Service | ledErrorMsg             | 1a9a8880-2305-11eb-adc1-0242ac120002 |

## IMPORTANT: Encoding to Write and Read the characteristics.
All messages you write or read should be encoded in ASCII unless otherwise noted.

@see <a href="annotated.html">Class list</a>