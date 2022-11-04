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

const mstyles = StyleSheet.create({
  tinyLogo: {
    width: 125,
    height: 65,
  },
  logo: {
    width: 330,
    height: 29,
  },
  container: {
    paddingTop: 10,
  },
  
});



const DisplayAnImage = ({ onClick, logo }) => {

  const theme = useTheme();
  const matches = useMediaQuery(theme.breakpoints.up('sm'));

  return (
    <View id="ist" style={(matches? styles.container: mstyles.container)}>

      
      <ImageBackground
        style={matches?styles.tinyLogo: mstyles.tinyLogo}
        source={logo} onClick={onClick}></ImageBackground>
      
    </View>
  );
}


export default DisplayAnImage;