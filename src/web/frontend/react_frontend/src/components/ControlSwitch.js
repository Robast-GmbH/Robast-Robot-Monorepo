import * as React from 'react';
import PropTypes from 'prop-types';
import Tabs from '@mui/material/Tabs';
import Tab from '@mui/material/Tab';
import Typography from '@mui/material/Typography';
import Box from '@mui/material/Box';
import RobotControl from './RobotControl';
import DrawerControl from './DrawerControl.js';
import  { useState } from "react";



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

export default function ControlSwitch( {Tablist}) {
  const [value, setValue] = React.useState(0);
  const [list, updateList] = Tablist;
  
  const handleChange = (event, newValue) => {
    setValue(newValue);
  };
  
  return (
    <div>
      <Box sx={{ width: '100%' }}>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={value} onChange={handleChange} aria-label="basic tabs example" centered={true}>
            {Tablist.map(item =>(
                <Tab key= {item.id} label={item.label} {...a11yProps(item.id)} />
              ))}
          </Tabs>
        </Box>
        {  Tablist.map(item =>(
                <TabPanel value={value} index={item.id}>
                   {item.content}
               </TabPanel>
              ))}
        
      </Box>
    </div>
  );
}