import * as React from 'react';
import PropTypes from 'prop-types';
import Tabs from '@mui/material/Tabs';
import Tab from '@mui/material/Tab';
import Typography from '@mui/material/Typography';
import Box from '@mui/material/Box';
import Stack from '@mui/material/Stack';
import { useTheme } from '@mui/material/styles';
import useMediaQuery from '@mui/material/useMediaQuery';



function TabPanel(props) {
  const { children, value, index, ...other } = props;

  return (
    <div
      role="tabpanel"
      style={ {flexGrow: 1}}
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
  const [value, setValue] = React.useState(sessionStorage.getItem('tap-value')===1?1:0);
 
 
  
  const handleChange = (event, newValue) => {
    setValue(newValue);
    sessionStorage.setItem('tap-value', newValue)
  };

  const theme = useTheme();
  const matches = useMediaQuery(theme.breakpoints.up('sm'));
  
  return (
   

    <Stack direction={ matches ? "row" : "column" } >
      <Tabs
        orientation={ matches ? "vertical" : "horizontal" }
        variant="scrollable"
        value={value}
        onChange={handleChange}
        aria-label="Vertical tabs example"
        sx={{ borderRight: 0, borderColor: 'divider' }}
      >
        {Tablist.map(item =>( <Tab key= {item.id} label={item.label} {...a11yProps(item.id)} /> ))}
      </Tabs>
      {  Tablist.map(item =>(
                <TabPanel key={item.id}  value={value} index={item.id}>
                   {item.content}
               </TabPanel>
              ))}
      
    </Stack>
  );
}
     