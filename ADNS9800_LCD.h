#include "ADNS9800.h"

const int16_t x_scale = 56;
const int16_t y_scale = 56;

class adns_ctrl : 
public adns::controller {
public:
    adns_ctrl() : 
    _lcd_key(0), _adc_key_in(0), _lcd(8, 9, 4, 5, 6, 7) {
    }
    ~adns_ctrl() {
    }

    void setup();
    void loop();

    void get_xy(int16_t x, int16_t y);
    void get_xy_dist(int16_t x_sum, int16_t y_sum);
    void get_squal(uint16_t s);
    void get_fault();
    void clear();

    // LCD buttons
    void btnNONE();
    void btnRIGHT();
    void btnUP();
    void btnDOWN();
    void btnLEFT();
    void btnSELECT();

private:
    void read_lcd_buttons();
    void print_xy_serial();
    void print_xy_dist_serial();
    void print_xy_lcd();
    void print_xy_dist_lcd();
    void print_squal_lcd();

private:
    int16_t _x;
    int16_t _y;
    int16_t _x_dist;
    int16_t _y_dist;
    uint16_t _squal;
    int _lcd_key;
    int _adc_key_in;
    LiquidCrystal _lcd;
};

void adns_ctrl::get_xy(int16_t x, int16_t y)
{
    _x = x;
    _y = y;
}

void adns_ctrl::get_xy_dist(int16_t x_dist, int16_t y_dist)
{
    _x_dist = x_dist;
    _y_dist = y_dist;
    print_xy_dist_serial();
    print_xy_dist_lcd();
}

void adns_ctrl::get_squal(uint16_t s)
{
    _squal = s;
    print_squal_lcd();
}

void adns_ctrl::get_fault()
{
    Serial.println("XY_LASER is shorted to GND");
    _lcd.print("FAULT");
}

void adns_ctrl::clear()
{
    _lcd.clear();
}

// read the buttons
void adns_ctrl::read_lcd_buttons()
{
    _adc_key_in = analogRead(0);      // read the value from the sensor
    // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
    // we add approx 50 to those values and check to see if we are close
    if (_adc_key_in > 1000) btnNONE(); // We make this the 1st option for speed reasons since it will be the most likely result
    else if (_adc_key_in < 50)   btnRIGHT(); 
    else if (_adc_key_in < 195)  btnUP();
    else if (_adc_key_in < 380)  btnDOWN();
    else if (_adc_key_in < 555)  btnLEFT();
    else if (_adc_key_in < 790)  btnSELECT();
}

void adns_ctrl::print_xy_serial()
{
    Serial.print("x = ");
    Serial.print(_x / x_scale);
    Serial.print(" | ");
    Serial.print("y = ");
    Serial.println(_y / y_scale);
}

void adns_ctrl::print_xy_dist_serial()
{
    Serial.print("x = ");
    Serial.print(_x_dist / x_scale);
    Serial.print(" | ");
    Serial.print("y = ");
    Serial.println(_y_dist / y_scale);
}

void adns_ctrl::print_xy_lcd()
{
    _lcd.setCursor(0, 0);
    _lcd.print("x:");
    _lcd.print(_x / x_scale);

    _lcd.setCursor(8, 0);
    _lcd.print("y:");
    _lcd.print(_y / y_scale);
}

void adns_ctrl::print_xy_dist_lcd()
{
    _lcd.setCursor(0, 0);
    _lcd.print("x:");
    _lcd.print(_x_dist / x_scale);

    _lcd.setCursor(8, 0);
    _lcd.print("y:");
    _lcd.print(_y_dist / y_scale);
}

void adns_ctrl::print_squal_lcd()
{
    _lcd.setCursor(0, 1);
    _lcd.print("SQUAL:");
    _lcd.print(_squal);
}

void adns_ctrl::btnNONE()
{
}

void adns_ctrl::btnRIGHT()
{
    _lcd.setCursor(0, 1);
    _lcd.print("RIGHT           ");
}

void adns_ctrl::btnUP()
{
    _lcd.setCursor(0, 1);
    _lcd.print("UP              ");
}

void adns_ctrl::btnDOWN()
{
    _lcd.setCursor(0, 1);
    _lcd.print("DOWN            ");
}

void adns_ctrl::btnLEFT()
{
    _lcd.setCursor(0, 1);
    _lcd.print("LEFT            ");
}

void adns_ctrl::btnSELECT()
{
    _lcd.clear();
    reset_xy_dist();
    print_xy_lcd();
    _lcd.setCursor(0, 1);
    _lcd.print("SELECT = RESET  ");
}

void adns_ctrl::setup()
{
    controller::setup();
    _lcd.begin(16, 2);
    _lcd.clear();
}

void adns_ctrl::loop()
{
    controller::loop();
    read_lcd_buttons();
}

