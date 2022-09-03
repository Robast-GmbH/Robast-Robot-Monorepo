import PropTypes from 'prop-types'
import * as React from 'react';
import Button from '@mui/material/Button';
import ButtonGroup from '@mui/material/ButtonGroup';



const CallRobot = () => {     
        return (
                <>
                 <ButtonGroup variant="contained" aria-label="outlined primary button group" id="callOptions" >
                    <Button>A</Button>
                    <Button>B</Button>
                    <Button>C</Button>
                    <Button>D</Button>
                </ButtonGroup>  
                </>
        )
}


CallRobot.protoTypes = {
      
}

export default CallRobot
