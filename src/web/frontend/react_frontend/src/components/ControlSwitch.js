import * as React from 'react';
import PropTypes from 'prop-types';
import Tabs from '@mui/material/Tabs';
import Tab from '@mui/material/Tab';
import Typography from '@mui/material/Typography';
import Box from '@mui/material/Box';
import RobotControl from './RobotControl';
import DrawerControl from './DrawerControl.js';
import  { useState } from "react";

const TabData=[
  { id:0, content:  <RobotControl/> , label: "Roboter Steuern"},
  { id:1, content:  <DrawerControl/> , label: "Schubladen Öffnen"}
  ];

function TabPanel(props) {
  const { children, value, index, ...other } = props;

  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`simple-tabpanel-${index}`}
      aria-labelledby={`simple-tab-${index}`}
      {...other}
    >
      {value === index && (
        <Box sx={{ p: 3 }}>
          <Typography>{children}</Typography>
        </Box>
      )}
    </div>
  );
}

TabPanel.propTypes = {
  children: PropTypes.node,
  index: PropTypes.number.isRequired,
  value: PropTypes.number.isRequired,
};

function a11yProps(index) {
  return {
    id: `simple-tab-${index}`,
    'aria-controls': `simple-tabpanel-${index}`,
  };
}


export default function ControlSwitch() {
  const [value, setValue] = React.useState(0);
  const [list, updateList] = TabData;

  const handleChange = (event, newValue) => {
    setValue(newValue);
  };

  return (
    <div>
      <Box sx={{ width: '100%' }}>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={value} onChange={handleChange} aria-label="basic tabs example" centered={true}>
            { TabData.map(item =>(
                <Tab label={item.label} {...a11yProps(item.id)} />
              ))}
          </Tabs>
        </Box>
        { TabData.map(item =>(
                <TabPanel value={value} index={item.id}>
                   {item.content}
               </TabPanel>
              ))}
        
      </Box>
    </div>
  );
}