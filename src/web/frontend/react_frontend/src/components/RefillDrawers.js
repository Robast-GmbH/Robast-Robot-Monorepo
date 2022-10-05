
import RefillDrawer from './RefillDrawer';
import Stack from '@mui/material/Stack';


const RefillDrawers = ({drawers, toggleEmpty}) => {
    return (
                <Stack spacing={2}>
                    <h2 id="titel_rename">Schubladen Füllstände</h2>
                    {drawers.map((drawer) => (
                        
                        <RefillDrawer key={drawer.id} drawer = {drawer} toggleEmpty = {toggleEmpty} />
                    ))}
                </Stack>
            
    )
}

export default RefillDrawers
            