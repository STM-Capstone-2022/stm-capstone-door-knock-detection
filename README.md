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

## Setting up the alert system

For the alert, we will be using the rules engine, a free service that monitors MQTT messages and triggers an action once it detects the correct message. The rule will send an email as soon as the rule is triggered.

To setup the rule, on the IoT Core navigate to the left side bar and go to Manage -> Message Routing -> Rules.

This window will show you all the rules you have created. It's important to note that like things, rules are also region dependant, so make sure to create the rule on the same region as your device. To create the rule, click on the orange **create rule** button on the top right.

### Step 1
The first step is to give it a name. Descriptions and tags are optional and is not needed for this.

### Step 2
The second step is the SQL statement. This is the heart of the rule, where it determines what message to look out for in a subscribed topic. For this, keep the SQL version to 2016-03-23. 

The statement should be written out as follows

![image](https://user-images.githubusercontent.com/59811685/183310426-21c4caef-21da-443f-ad99-7b4dcecc06bc.png)

In this case, the SQL statement is looking at the double_tap object in the payload from the stm32test4/motion_sensor_data topic. Note that stm32test4 should be changed to represent your device's name. The last part says that it will only trigger the rule if double_tap is true.

### Step 3

For step 3, we will attach an action to the rule. The action defines what happens when the rule is triggered

For this, we only need one action rule. Click on the drop down menu and select the action **Simple Notification Service (SNS)**

For the SNS topic, we want to create a new SNS topic by clicking on the button to the right, where a new window will open

### Step 3.5: Set Up The Message System

Set the topic type to be standard and give the topic a name, preferably the way the message will be sent (through EMail). Nothing else needs to be altered, so create the topic

![asdf](https://user-images.githubusercontent.com/59811685/183310806-482803f4-3b49-42fb-9430-f9f2cf036f65.JPG)
