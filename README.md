# Door Knock Detection Alert through AWS IoT Core

This repository allows you to change your project code to send a door knock event to AWS IoT Core, which you can use to send an alert to the user.

Within this repository contains the necessary files to change the code to detect a door knock through the built-in double tap feature.

## Prerequisits

The only prerequisit is to follow the set up tutorial [linked here](https://github.com/FreeRTOS/iot-reference-stm32u5)

## Changing the code

Navigate to your root directory for the Project. If you compare the directory names within the project to the ones on the repo, you should notice they have the same names. This is done to make it easier to find the right files.

***Create a backup of the original project at this point if you haven't done so already. The project is prone to not work if files are not transfered correctly.***

Use the file paths on the repo to find the correct files to be transfered.

## Testing the Door Knock Feature

Rebuild and reflash your code. If all the files were transfered correctly there will be no build errors.

To make sure that it is running correctly, log back into [aws.amazon.com](aws.amazon.com) with your IAM user and navigate back to the MQTT test client.

Subscribe to the topic *name of device given from provision script*/motion_sensor_data

Double tap the device to see the MQTT payload. The device doesn't react if the taps are too fast or too slow, so it's recommended to tap with about a half second interval. Below if a picture of what the payload should look like.

![image](https://user-images.githubusercontent.com/59811685/183277917-240e6c6b-5324-4f96-bc76-609c46a89193.png)
