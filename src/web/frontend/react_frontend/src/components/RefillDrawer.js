
import * as React from 'react';
import PropTypes from 'prop-types'

import ToggleButton from '@mui/material/ToggleButton';
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup';

import NoDrinksIcon from '@mui/icons-material/NoDrinks';
import LocalBarIcon from '@mui/icons-material/LocalBar';

import Stack from '@mui/material/Stack';

const RefillDrawer = ({ drawer, toggleEmpty }) => {
        const [isEmtpty, setIsEmtpty] = React.useState(drawer.empty);
    return(  
                <Stack direction="row" spacing={5} id= "emptySelector" >
                        <label width="20%">{drawer.content+" :"}</label>   
                        <ToggleButtonGroup
                                value={isEmtpty}
                                exclusive
                                onChange={(e, state)=> {setIsEmtpty(state)
                                                        toggleEmpty(drawer)
                                                        }}
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


