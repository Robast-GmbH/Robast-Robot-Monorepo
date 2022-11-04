
import * as React from 'react';
import PropTypes from 'prop-types'

import ToggleButton from '@mui/material/ToggleButton';
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup';
import InputLabel from '@mui/material/InputLabel';

import NoDrinksIcon from '@mui/icons-material/NoDrinks';
import LocalBarIcon from '@mui/icons-material/LocalBar';

import Stack from '@mui/material/Stack';



const RefillDrawer = ({ drawer, toggleEmpty }) => {
        const [isEmtpty, setIsEmtpty] = React.useState(drawer.empty);


        const handleRefill = (event, newRefill) => {
                if (newRefill !== null) {
                        setIsEmtpty(newRefill)
                        toggleEmpty(drawer)
                }
        };
    return(  
                <Stack direction="row" spacing={5} id= "emptySelector" justify-content= "space-between">
                        <InputLabel id="titel_rename">{drawer.content+" :"} </InputLabel>
                        <ToggleButtonGroup
                                value={isEmtpty}
                                exclusive
                                onChange= {handleRefill}
                                aria-label="text alignment"
                                >
                
                                <ToggleButton value= {false} aria-label="filled" color="primary">
                                        <LocalBarIcon />
                                </ToggleButton>
                                <ToggleButton value= {true} aria-label="empty" color= "error">
                                        <NoDrinksIcon />
                                </ToggleButton>
        
                        </ToggleButtonGroup>
                </Stack>
        )
}
export default RefillDrawer
RefillDrawer.propTypes = {
        drawer: PropTypes.object,
        toggleEmpty : PropTypes.func,
}


