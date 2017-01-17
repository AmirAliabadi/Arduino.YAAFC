void store_in_eeprom()
{
  EEPROMData data_to_put = {
    "AA",    
    ax_offset,
    ay_offset,
    az_offset,
    gx_offset,
    gy_offset,
    gz_offset
  };

  int eeAddress = 0;   //Location we want the data to be put
  EEPROM.put(eeAddress, data_to_put);

  EEPROMData data_valid ;
  EEPROM.get(eeAddress, data_valid);
  Serial.print("EEPROM Updated:\t");
  Serial.print(data_valid.ax_offset);
  Serial.print("\t");
  Serial.print(data_valid.ay_offset);
  Serial.print("\t");
  Serial.print(data_valid.az_offset);
  Serial.print("\t");
  Serial.print(data_valid.gx_offset);
  Serial.print("\t");
  Serial.print(data_valid.gy_offset);
  Serial.print("\t");
  Serial.println(data_valid.gz_offset);
 
}


