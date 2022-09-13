import React from 'react';
import { View, ImageBackground, StyleSheet } from 'react-native';
import { useTheme } from '@mui/material/styles';
import useMediaQuery from '@mui/material/useMediaQuery';

const styles = StyleSheet.create({
  tinyLogo: {
    width: 250,
    height: 130,
  },
  tinyLogoM: {

  },
  logo: {
    width: 660,
    height: 58,
  },
  container: {
    paddingTop: 10,
  },
  containerM: {

  }
});



const DisplayAnImage = ({ onClick, logo }) => {

  const theme = useTheme();
  const matches = useMediaQuery(theme.breakpoints.up('sm'));

  return (
    <View id="ist" style={styles.container}>

      
      <ImageBackground
        style={styles.tinyLogo}
        source={logo} onClick={onClick}></ImageBackground>
      
    </View>
  );
}


export default DisplayAnImage;