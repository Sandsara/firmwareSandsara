import processing.serial.*;
Serial mySerial;
long[] values; 
int picture_number = 0;

void data(long[] dvalor){
  String value = mySerial.readStringUntil(10);
  //print(value);
  if (value != null){
    if ( !Character.isDigit(value.charAt(0)) && value.charAt(0) != '-'){
      print(value);
      if (value.substring(0,8).equals("finished")){
        background(35);
        int s = second(); // Valores de 0 a 59
        int m = minute(); // Valores de 0 a 59
        int h = hour(); // Valores de 0 a 23
        int year = year(); // Valores de 2015, 2014, etc
        int month = month(); // Valores de 1 a 12
        int d = day(); // Valores de 1 a 31
        /*pushMatrix();
        segment(x, y, coordinates[1]);
        segment(segLength, 0, coordinates[2]);
        popMatrix();*/
        pushMatrix();
        segment(x, y, coordinates[1]);
        segment(segLength, 0, coordinates[2]);
        popMatrix();
        dataScreen.beginDraw();
        dataScreen.textSize(14);
        dataScreen.fill(255,0,0);
        dataScreen.text("Ends at: ", 10, 60);
        dataScreen.fill(255,255,0);
        dataScreen.text(h + ":" + m + ":" + s + ", " + d + "/" + month + "/" + year, 80, 60);
        dataScreen.translate( x, y);
        dataScreen.noStroke();
        dataScreen.fill(0, 0, 255);
        dataScreen.circle(x_draw, y_draw, 10);
        dataScreen.endDraw();
        translate(x , y);
        image(sandmark, -x, -y);
        image(sandmark, -x, -y);
        translate(-x , -y);
        image(dataScreen, 0, 0);
        //image(dataScreen, 0, 0);
        translate(x , y);
        noStroke();
        fill(0, 0, 255);
        //circle(x_draw, y_draw, 10);
        //translate(x, -y);
        noFill();
        strokeWeight(3);
        stroke(255);
        circle(0, 0, 310*factor);
        numberPoint = 0;
        save(picture_number + ".png");
        println("guardo el archivo: " + picture_number);
        picture_number += 1;
        dataScreen.clear();
        sandmark.clear();
        sandmark.beginDraw();
        sandmark.fill(255, 255, 255);
        sandmark.endDraw();
        
        }
        else if (value.substring(0,8).equals("fileName")){
        int s = second(); // Valores de 0 a 59
        int m = minute(); // Valores de 0 a 59
        int h = hour(); // Valores de 0 a 23
        int y = year(); // Valores de 2015, 2014, etc
        int month = month(); // Valores de 1 a 12
        int d = day(); // Valores de 1 a 31
        dataScreen.beginDraw();
        dataScreen.textSize(14);
        dataScreen.fill(255,0,0);
        dataScreen.text("file name: ", 10, 20);
        dataScreen.fill(255,255,0);
        dataScreen.text(value.substring(10), 80, 20);
        dataScreen.fill(255,0,0);
        dataScreen.text("Starts at: ", 10, 40);
        dataScreen.fill(255,255,0);
        dataScreen.text(h + ":" + m + ":" + s + ", " + d + "/" + month + "/" + y, 80, 40);
        dataScreen.endDraw();
        }
        dvalor[0] = 0;
        dvalor[1] = 0;
        dvalor[2] = 0;
        return;
      }
      int index = value.indexOf(",");
      String a = value.substring(index+1, value.length()-2);
      int index2= a.indexOf(",");
      
      while(index <= 0 || index2 <= 0){
      value = mySerial.readStringUntil(10);
      index = value.indexOf(","); 
      a = value.substring(index+1, value.length()-2);
      index2= a.indexOf(",");
      }
      
      String xstr= value.substring(0, index);
      String ystr = a.substring(0, index2);
      String zstr = a.substring(index2+1);
      //String zstr = a.substring(index2+1,a.length()-1);
      long xlong = Long.parseLong(xstr);
      long ylong = Long.parseLong(ystr);
      long zlong = Long.parseLong(zstr);
      dvalor[0] = xlong;
      dvalor[1] = ylong;
      dvalor[2] = zlong;
      numberPoint += 1;
    }
    else {
      dvalor[0] = 0;
      dvalor[1] = 0;
      dvalor[2] = 0;}
}
