# Smart health system  
## Project showcase  
In this project, I aim to create an smart health system. I use the Arduino UNO as the main microcontroller. This system can provide pluse measurement and breathe measurement.  

In my desired scenes, people are always busy with a lot of things such as presentations, tests and tons of homework. When they do this things, they always feel nervous and can't help being internal friction. My system can help them measure their pluse and breathe volume everywhere to remind them being clam. To reach this end, I use pluse sensor, breathe sensor and Lcd. When people breathe, they can see the change of wave on the lcd. When they breathe heavily, some text will appear on the lcd to remind them to be clam. At the same time, the lcd will show the pulse result. The led on pluse sensor will also blink with the frequency of our pluse.  

There is a image of our whole system:  
![image](https://user-images.githubusercontent.com/114272466/210099304-c44a1c2f-1998-45a2-a126-94844ef1fe69.png)  

## Project instructions  
### Components used  
Arduino Uno  
DFRobot heart rate sensor  
Belt  
LCD backpack  
Clip  
Adhesive tape  
Elastic band  
Needle  
Thread  
Variable resistor  
Resistor  
Scissors  

### Assembly details  
![image](https://user-images.githubusercontent.com/114272466/210101951-b74649e1-82cb-4325-ad60-634d45be0ec9.png)  

![image](https://user-images.githubusercontent.com/114272466/210102345-39a23cd2-c5e0-4bfd-85a1-762a3a9550e2.png)  

### Narrative overview of project development process
At the beginning I intend to use APDS 9960 to make my breathe sensor. However, the proximity function can not help me to measure pulse. The output of the APDS 9960 is always 18 although I change the gain and other parameters. Then, the laboratory assistant give me a pulse sensor so that I can finish the pulse measurement. THe result in serial is perfect. I am sorry that I use arduino. In fact, I want to use the RP2040 at begining, but I do not know how to use analog read on rp2040. Then I use Arduino to make this function.  
https://github.com/zizhanghan/final-project/blob/main/video/1.mp4  

For lcd, I learn some function from google to drive my lcd and show the result. Although it is not very clear, It can show the correct result.  
https://github.com/zizhanghan/final-project/blob/main/video/2.mp4  

For breathe sensor, I see a example from internet. They use belt, elastic band, thread, sliver thread and fiber conductor to finish their design.
However, I do not find the sliver thread and fiber conductor in shop. So I use wires and variable resistor which the resistence will change with the degree of bending. I try it and the result is perfect. I can see the breathe wave on the lcd.  
https://github.com/zizhanghan/final-project/blob/main/video/3.mp4  

### Troubleshooting
One of the problems I encountered is how to measure the pulse by using APDS 9960. After I try a lot of times, I think I can not use the proximity function to finish it. Only way is to use the receive function in APDS9960 to receive the result of outer light change. But I do not have much time because I have a lot of homeworks. At the same, the laboratory assistant borrow me a pulse sensor and I learn how to drive it. 

Another problem is how to make the breathe sensor. Because I can not find the conductor fiber which the resistence will change with the stretching degree。I use the variable resistor to take place it. Howver, the resistor can not stretch and I also want to see the change when the elastic band stretch. I propose that I make the resistor bend at begining. When I stretch the elastic band, the resistor will be straight and the resistence will change. Then I try this method and I make it. 

### Reflections on design and components
#### My design:
I made a safe doorbell system.

Pros:
It can measure the pulse and the led can blink with the frequency of people's heart rate.  
It also can show the breathe wave when people breathe and remind them if they are nervous and breathe heavily.  
It is convenient to bring.  

Cons:  
I couldn’t upload result to the website.  
THe lcd can not show the result clear. ( it is the problem of lcd quality).  
The result of pulse sensor should be more accurate. 

### Some components we want to recommend to other teams:  
Arduino UNO  
Pros:
It is suitable for the analog read.  
It has many library for us to use.  
Cons:  
Its code volume is limited in arduino.exe.  

Variable resistor  
Pros:  
The resistence will change with the degree of bending.  
The scale of resistance is enough.

Cons:  
Easily broken.  

### Other design we want to use next time:  
I would like use APDS 9960 to finish pulse measurement function again. I want to use extra led and read function of APDS 9960. Moreover, I want use another lcd to show the result more clear.  

Team Overview:  
https://github.com/zizhanghan  









