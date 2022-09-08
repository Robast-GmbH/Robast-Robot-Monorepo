import React from 'react';
import { View, ImageBackground, StyleSheet } from 'react-native';



const styles = StyleSheet.create({
  container: {
    paddingTop: 10,
  },
  tinyLogo: {
    width: 250,
    height: 130,
  },
  logo: {
    width: 660,
    height: 58,
  },
});

const DisplayAnImage = ({ onClick, logo, styleIn }) => {

  return (
    <View id="ist" style={styleIn ? styleIn : styles.container}>

      
      <ImageBackground
        style={styleIn ? styleIn.tinyLogo : styles.tinyLogo}
        source={logo} onClick={onClick}></ImageBackground>
      
    </View>
  );
}


export default DisplayAnImage;